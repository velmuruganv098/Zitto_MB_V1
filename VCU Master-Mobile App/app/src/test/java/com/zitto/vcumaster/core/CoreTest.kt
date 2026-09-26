package com.zitto.vcumaster.core

import com.zitto.vcumaster.link.SimLink
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.cancel
import kotlinx.coroutines.delay
import kotlinx.coroutines.runBlocking
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Test
import java.util.Collections

/**
 * Checks the Kotlin port against the desktop VCU Master:
 *  - DBC decode/encode vs cantools 44.1 reference frames (Intel, Motorola, signed, multiplexed)
 *  - parser vs the type/tags the Python app recorded from real bench logs
 *  - vehicle auto-map vs dbc_engine.py
 *  - simulator end to end (streams, OTA, flash, modules) through parser + DBC engine
 */
class CoreTest {
    private fun res(n: String) = javaClass.classLoader!!.getResource(n)!!.readText()

    @Test
    fun dbcDecodeAndEncodeMatchCantools() {
        val dbs = listOf("zitto_demo_vehicle.dbc", "motorola_test.dbc").associateWith { DbcParser.parse(res(it)) }
        var n = 0
        for (line in res("cantools_ref.tsv").lines().filter { it.isNotBlank() }) {
            val (dbc, msg, hex, vals) = line.split("\t")
            val m = dbs.getValue(dbc).byName.getValue(msg)
            val data = hex.chunked(2).map { it.toInt(16) }.toIntArray()
            val exp = vals.split(";").associate { kv -> kv.substringBefore("=") to kv.substringAfter("=").toDouble() }
            val dec = m.decode(data)
            assertEquals("$msg $hex signal set", exp.keys, dec.keys)
            for ((k, v) in exp) assertEquals("$msg.$k from $hex", v, dec.getValue(k), 1e-6)
            if (m.signals.none { it.muxSwitch }) {
                assertEquals("$msg encode", hex, m.encode(exp).joinToString("") { "%02x".format(it) })
            }
            n++
        }
        assertEquals(175, n)
    }

    @Test
    fun dbcMetadata() {
        val db = DbcParser.parse(res("zitto_demo_vehicle.dbc"))
        assertEquals(5, db.messages.size)
        val odo = db.byName.getValue("Dash_Odometer")
        assertTrue(odo.ext)
        assertEquals(0x18FF5010L, odo.frameId)
        val gear = db.byName.getValue("VCU_State").signals.first { it.name == "Gear" }
        assertEquals(mapOf(0L to "P", 1L to "D", 2L to "R", 3L to "N"), gear.choices)
        assertEquals("State of charge from coulomb counter", db.byName.getValue("BMS_Pack").signals.first { it.name == "SOC" }.comment)
        assertEquals("Pack level measurements, 20 Hz", db.byName.getValue("BMS_Pack").comment)
    }

    @Test
    fun vehicleAutomapMatchesDesktop() {
        val e = DbcEngine()
        e.load("d.dbc", res("zitto_demo_vehicle.dbc"), listOf(1, 2))
        val expected = mapOf(
            "speed" to "MCU_Status.VehicleSpeed", "motor_rpm" to "MCU_Status.MotorSpeed_rpm", "soc" to "BMS_Pack.SOC",
            "pack_voltage" to "BMS_Pack.PackVoltage", "pack_current" to "BMS_Pack.PackCurrent",
            "batt_temp" to "BMS_Cells.CellTmax", "motor_temp" to "MCU_Status.MotorTemp",
            "ctrl_temp" to "MCU_Status.ControllerTemp", "throttle" to "VCU_State.Throttle", "brake" to "VCU_State.Brake",
            "gear" to "VCU_State.Gear", "odometer" to "Dash_Odometer.Odometer", "fault" to "VCU_State.VCU_Fault",
        )
        assertEquals(expected, e.vehicleMap.toMap())
    }

    @Test
    fun parserMatchesDesktopOnBenchLogs() {
        var n = 0
        for (line in res("bench_lines.tsv").lines().filter { it.isNotBlank() }) {
            val (type, tags, raw) = line.split("\t", limit = 3)
            if (type == "TX" || type == "TX_ERR") continue
            val p = Parser.parse(raw)
            assertEquals(raw, type, p.type)
            assertEquals(raw, tags.split("|"), p.tags)
            n++
        }
        assertTrue("bench lines parsed: $n", n > 400)
    }

    @Test
    fun parserFieldsMatchDesktop() {
        val st = Parser.parse("seq=9 STATUS imu=1 csa=1 can1=1 can2=1 flm=1 ota=0 can1_baud=500 can2_baud=250 flash_free=60000 uptime=1234 reset=0x80 hb=3")
        assertEquals(listOf("STATUS", "SYSTEM"), st.tags)
        assertEquals(128L, st.fields["reset"]); assertEquals("POR", st.fields["reset_name"]); assertEquals(9, st.seq)
        val cs = Parser.parse("seq=4 CAN_STATUS bus=2 state=3 ready=1 bus_off=0 baud=250 rx=5 err=0 tx_err=0 rx_err=0 irq=1 err_irq=0 mb_irq=1 ts=5ms")
        assertEquals("RUNNING", cs.fields["state_name"]); assertEquals(listOf("CAN2", "CAN_STATUS"), cs.tags); assertEquals(5L, cs.fields["ts"])
        val fd = Parser.parse("seq=5 FLASH_DATA len=5 hex=68656C6C6F")
        assertEquals("68656C6C6F", fd.fields["hex"]); assertEquals(false, fd.fields["empty"]); assertEquals(5L, fd.fields["len"])
        val imu = Parser.parse("seq=6 IMU accel_mg=(12,-3,1001) gyro_mdps=(1500,-20,7) temp=31.2C ts=812ms")
        assertEquals(-3L, imu.fields["ay_mg"]); assertEquals(31.2, imu.fields["temp_c"]); assertEquals(812L, imu.fields["ts_ms"])
        val can = Parser.parse("seq=7 CAN bus=1 id=0x18ff0010 EXT DATA dlc=8 data=[01 02 03 04 05 06 07 08] ts=9ms")
        assertEquals(0x18FF0010L, can.fields["id"]); assertEquals(true, can.fields["ext"]); assertEquals(listOf("CAN1", "CAN"), can.tags)
        assertEquals(listOf(1, 2, 3, 4, 5, 6, 7, 8), can.fields["data"])
    }

    @Test
    fun protocolCommands() {
        assertEquals("RAW:010001", Protocol.moduleEn("IMU", true))
        assertEquals("RAW:05F4013200", Protocol.ledCtrl(500, 50))
        assertEquals("RAW:10000003E8CBF43926", Protocol.otaStart(1000, 0xCBF43926L))
        assertEquals(0xCBF43926L, Protocol.crc32("123456789".toByteArray()))
        assertEquals("RAW:020D0101", Protocol.s32GpioRaw(13, 1, 1))
        assertEquals(0x07 to listOf<Byte>(0x68, 0x69), Protocol.parseRaw("RAW:076869").let { it.first to it.second.toList() })
    }

    @Test
    fun simulatorEndToEnd() = runBlocking {
        val db = DbcParser.parse(res("zitto_demo_vehicle.dbc"))
        val lines = Collections.synchronizedList(mutableListOf<String>())
        val scope = CoroutineScope(Dispatchers.Default + SupervisorJob())
        val sim = SimLink(scope, db)
        sim.onLine = { lines += it }
        sim.connect("SIM", null)
        delay(5300)

        val img = ByteArray(1000) { (it * 7).toByte() }
        sim.write(Protocol.otaStart(img.size, Protocol.crc32(img)))
        for (c in img.toList().chunked(96)) sim.write(Protocol.otaData(c.toByteArray()))
        sim.write(Protocol.otaFinish())
        sim.write(Protocol.flashWrite("hello".toByteArray()))
        sim.write(Protocol.flashRead())
        sim.write(Protocol.moduleEn("IMU", false))
        sim.write(Protocol.ledCtrl(250, 30))
        sim.write(Protocol.s32GpioRaw(13, 1, 1))
        sim.write("PING")
        delay(200)
        sim.disconnect()
        scope.cancel()

        val all = lines.toList()
        val recs = all.map { Parser.parse(it) }
        assertTrue(recs.none { "PARSE_ERR" in it.tags })
        val types = recs.map { it.type }.toSet()
        for (t in listOf("IMU", "CSA", "CAN", "CAN_STATUS", "HEARTBEAT", "STATUS", "BRIDGE_STATUS", "INFO", "PONG", "FLASH_DATA", "GPIO_STATUS", "CMD_ACK")) {
            assertTrue("missing $t", t in types)
        }
        assertTrue("LOG OTA:start_ok" in all.map { it.substringAfter(' ') })
        assertTrue(all.any { it.endsWith("LOG OTA:ok") })
        assertTrue(all.any { it.endsWith("LOG [CMD] LED period=250 ms duty=30 %  OK") })
        // V0.0073: module command -> "[CMD]" event (tagged CMD) + CMD_ACK cmd=0x1 with the module id and new state
        val modAck = recs.last { it.type == "CMD_ACK" && it.fields.long("cmd") == 1L }
        assertEquals(0L, modAck.fields["result"]); assertEquals(0L, modAck.fields["gpio_id"]); assertEquals(0L, modAck.fields["state"])
        assertTrue(recs.any { it.type == "LOG" && "CMD" in it.tags && (it.fields["text"] as String).startsWith("[CMD] MODULE IMU -> DISABLED") })
        // V0.0073: IMU displacement fields every 100 ms
        val imu = recs.filter { it.type == "IMU" }
        assertTrue("imu ${imu.size}", imu.size >= 40)
        assertTrue(imu.all { it.fields["pos_x_mm"] is Double && it.fields["moving"] is Long && it.fields["imu_up_ms"] is Long })
        assertEquals("68656C6C6F", recs.last { it.type == "FLASH_DATA" }.fields["hex"])
        assertEquals(0L, recs.last { it.type == "STATUS" }.fields["imu"])
        assertEquals(1, (recs.last { it.type == "GPIO_STATUS" }.fields["pins"] as List<*>).map { it as GpioPin }.first { it.id == 13 }.state)

        val eng = DbcEngine()
        eng.load("d.dbc", res("zitto_demo_vehicle.dbc"), listOf(1, 2))
        var decoded = 0
        for (r in recs.filter { it.type == "CAN" && eng.dbcs.getValue("d.dbc").db.byId.containsKey(it.fields.long("id")) }) {
            @Suppress("UNCHECKED_CAST")
            val d = eng.decode(r.fields.long("bus").toInt(), r.fields.long("id"), r.fields["ext"] == true, r.fields["data"] as List<Int>, r.t)
            assertNotNull(d)
            assertNull(d!!.error)
            decoded++
        }
        assertTrue("decoded $decoded", decoded > 100)
        val veh = eng.vehicleSnapshot(Fmt.nowS())
        val speed = veh.getValue("speed").value!!
        assertTrue("speed $speed", speed in 0.0..60.0)
        val soc = veh.getValue("soc").value!!
        assertTrue("soc $soc", soc in 80.0..90.0)
        assertEquals("D", veh.getValue("gear").text)
        assertTrue(eng.messages.keys.any { it == "2:0x18FF5010" })

        // V0.0073: the simulated Daly BMS on CAN2 is unknown to the demo DBC -> library auto-match -> Battery roles
        fun feed() {
            for (r in recs.filter { it.type == "CAN" }) {
                @Suppress("UNCHECKED_CAST")
                eng.decode(r.fields.long("bus").toInt(), r.fields.long("id"), r.fields["ext"] == true, r.fields["data"] as List<Int>, Fmt.nowS())
            }
        }
        feed()
        val unk = eng.unknownIds(Fmt.nowS())
        assertTrue(unk.keys.toString(), (0x18954001L to true) in unk.keys && unk.values.all { it == setOf(2) })
        val lib = DbcLibrary(java.io.File("src/main/assets/library_index.json").readText())
        val best = lib.match(unk.keys).first()
        assertEquals("BMS/Daly/Daly_BMS_CAN_V1.0_from_spec.dbc", best.id)
        eng.load("daly.dbc", java.io.File("src/main/assets/dbc_library/${best.id}").readText(Charsets.ISO_8859_1), listOf(2))
        feed()
        assertTrue(eng.unknownIds(Fmt.nowS()).isEmpty())
        val meta = eng.rolesMeta()
        assertTrue("battery" in meta.panels && "motor" in meta.panels)
        assertEquals(16.0, eng.roleValues.getValue("bms.cell_count"), 0.0)
        assertEquals(4.0, eng.roleValues.getValue("bms.temp_count"), 0.0)
        for (k in meta.cells.take(16)) assertTrue(k, eng.roleValues.getValue(k) in 3.2..3.4)
        assertTrue(meta.cells.drop(16).all { (eng.roleValues[it] ?: 0.0) == 0.0 })     // unfitted: absent or 0 mV
        assertTrue(eng.roleValues.getValue("bms.soc") in 80.0..90.0)
        assertEquals(1.0, eng.roleValues.getValue("bms.charge_mos"), 0.0)
        assertTrue(eng.roleValues.getValue(meta.temps[0]) in 20.0..40.0)
    }
}
