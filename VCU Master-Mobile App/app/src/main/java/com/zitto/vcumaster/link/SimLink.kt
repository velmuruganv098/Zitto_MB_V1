package com.zitto.vcumaster.link

import com.zitto.vcumaster.core.DbcDatabase
import com.zitto.vcumaster.core.Protocol
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.util.Random
import kotlin.math.cos
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

/** Emulates the S32K144 firmware + ESP32 bridge text output (port of links.py SimLink). */
class SimLink(private val scope: CoroutineScope, private val db: DbcDatabase?) : Link() {
    override val kind = "sim"

    private var job: Job? = null
    private var seq = 0
    private var t0 = now()
    private val mods = linkedMapOf("IMU" to 1, "CSA" to 1, "CAN1" to 1, "CAN2" to 1, "FLM" to 1)
    private val gpio = (1..13).associateWith { intArrayOf(0, 0) }.toMutableMap()   // dir, state
    private val espGpio = HashMap<Int, Int>()
    private val flash = ArrayList<ByteArray>()
    private var otaActive = false
    private var otaPending = false
    private var otaSize = 0
    private var otaCrc = 0L
    private var otaBuf = java.io.ByteArrayOutputStream()
    private var hb = 0
    private var frames = 0
    private val rnd = Random()

    init { mtu = 247 }

    private fun up(): Long = ((now() - t0) * 1000).toLong()
    private fun gauss(sd: Double) = rnd.nextGaussian() * sd
    private fun uni(a: Double) = (rnd.nextDouble() * 2 - 1) * a

    private fun pub(body: String, framed: Boolean = true) {
        var b = body
        if (framed) {
            seq = (seq + 1) and 0xFF
            frames++
            b = "seq=$seq $body"
        }
        emitLine(b.take(200))
    }

    override suspend fun connect(address: String, name: String?) {
        this.address = "SIM:00:00:00"
        this.name = "Zitto_MB_V1_Bridge (simulated)"
        connected = true
        connectedAt = now()
        error = null
        emitState()
        job = scope.launch { run() }
        pub("BLE_CONNECTED", framed = false)
        pub("INFO Zitto_MB_V1_Bridge UART2=115200 BLE=ON", framed = false)
    }

    override suspend fun disconnect(user: Boolean) {
        job?.cancel()
        job = null
        connected = false
        emitState()
    }

    // ------------------------------------------------------------ streams
    private fun status() {
        val m = mods
        pub(
            "STATUS imu=${m["IMU"]} csa=${m["CSA"]} can1=${m["CAN1"]} can2=${m["CAN2"]} " +
                "flm=${m["FLM"]} ota=${if (otaPending) 1 else 0} can1_baud=500 " +
                "can2_baud=250 flash_free=${60000 - flash.size} uptime=${up()} reset=0x80 hb=$hb",
        )
    }

    private fun gpioStatus() {
        val parts = gpio.entries.joinToString(" ") { (i, g) -> "#$i:${if (g[0] == 1) "OUT" else "IN"}=${g[1]}" }
        pub("GPIO_STATUS $parts ")
    }

    private fun can(bus: Int, msgName: String, values: Map<String, Double>) {
        val m = db?.byName?.get(msgName) ?: return
        val full = HashMap<String, Double>()
        for (s in m.signals) {
            var v = values[s.name] ?: 0.0
            s.min?.let { v = max(it, v) }
            s.max?.let { v = min(it, v) }
            full[s.name] = v
        }
        val data = m.encode(full)
        val ext = if (m.ext) " EXT" else " STD"
        val hexd = data.joinToString(" ") { "%02x".format(it) }
        pub("CAN bus=$bus id=0x${m.frameId.toString(16)}$ext DATA dlc=${data.size} data=[$hexd] ts=${up()}ms")
    }

    private suspend fun run() {
        var k = 0
        var odo = 1234.5
        var soc = 86.0
        while (scope.isActive) {
            delay(50)   // 50 ms tick like TASK_DT_MS
            k++
            val t = now() - t0
            val speed = max(0.0, 32 + 18 * sin(t / 7.0) + uni(0.6))
            val rpm = speed * 95
            val current = 8 + speed * 1.6 + uni(2.0)
            soc = max(5.0, soc - current * 0.000015)
            odo += speed / 3600 * 0.05

            if (mods["IMU"] == 1 && k % 10 == 0) {
                val ax = (35 * sin(t * 1.3) + gauss(6.0)).toInt()
                val ay = (-20 + 25 * cos(t * 0.9) + gauss(6.0)).toInt()
                val az = (1000 + gauss(8.0)).toInt()
                val gx = (1500 * sin(t * 0.7) + gauss(60.0)).toInt()
                val gy = (800 * cos(t * 1.1) + gauss(60.0)).toInt()
                val gz = (2500 * sin(t / 3.0) + gauss(60.0)).toInt()
                val tc = 31.0 + 1.5 * sin(t / 60)
                pub("IMU accel_mg=($ax,$ay,$az) gyro_mdps=($gx,$gy,$gz) temp=${"%.1f".format(java.util.Locale.US, tc)}C ts=${up()}ms")
            }
            if (mods["CSA"] == 1 && k % 4 == 0) {
                val ma = (420 + 60 * sin(t / 5) + gauss(8.0)).toInt()
                val mv = (12050 + gauss(15.0)).toInt()
                pub("CSA current=${ma}mA voltage=${mv}mV power=${ma.toLong() * mv / 1000}mW ts=${up()}ms")
            }
            if (mods["CAN1"] == 1 && db != null) {
                if (k % 2 == 0) can(1, "MCU_Status", mapOf(
                    "MotorSpeed_rpm" to rpm, "VehicleSpeed" to speed,
                    "MotorTemp" to 48 + speed * 0.3, "ControllerTemp" to 41 + speed * 0.2,
                    "MotorTorque" to current * 0.35,
                ))
                if (k % 2 == 1) can(1, "BMS_Pack", mapOf(
                    "PackVoltage" to 51.2 - current * 0.012, "PackCurrent" to current,
                    "SOC" to soc, "SOH" to 97.0,
                ))
                if (k % 10 == 3) can(1, "BMS_Cells", mapOf(
                    "CellVmax" to 3.36, "CellVmin" to 3.31 - current * 0.0004,
                    "CellTmax" to 29 + current * 0.05, "CellTmin" to 27.0,
                ))
            }
            if (mods["CAN2"] == 1 && db != null) {
                if (k % 4 == 1) {
                    val thr = (speed * 2.1).coerceIn(0.0, 100.0)
                    can(2, "VCU_State", mapOf(
                        "Gear" to if (speed > 1) 1.0 else 0.0, "Throttle" to thr,
                        "Brake" to if (t % 23 < 2) 1.0 else 0.0, "DriveMode" to 1.0,
                        "VCU_Fault" to 0.0, "KeyOn" to 1.0,
                    ))
                }
                if (k % 20 == 7) can(2, "Dash_Odometer", mapOf("Odometer" to odo, "TripA" to odo - 1200))
            }
            if (k % 20 == 0) {
                val u = up()
                if (mods["CAN1"] == 1) pub(
                    "CAN_STATUS bus=1 state=1 ready=1 bus_off=0 baud=500 rx=$rxLines err=0 tx_err=0 " +
                        "rx_err=0 irq=${k * 3} err_irq=0 mb_irq=${k * 3} ts=${u}ms",
                )
                if (mods["CAN2"] == 1) pub(
                    "CAN_STATUS bus=2 state=3 ready=1 bus_off=0 baud=250 rx=${k / 4} err=0 tx_err=0 " +
                        "rx_err=0 irq=$k err_irq=0 mb_irq=$k ts=${u}ms",
                )
            }
            if (k % 100 == 0) {
                hb++
                pub("HEARTBEAT uptime=${up()}ms")
                status()
                pub("BRIDGE_STATUS uart_frames=$frames crc_errors=0 ble=1", framed = false)
            }
            if (k % 200 == 50 && mods["FLM"] == 1) flm()
        }
    }

    private fun flm() {
        val used = flash.size
        pub("FLM total=61440 used=$used free=${61440 - used} next=${64 + used} last=${63 + used} records=$used ts=${up()}ms")
    }

    // ------------------------------------------------------------ commands
    override suspend fun write(cmd: String) {
        if (!connected) throw LinkException("simulator not connected")
        txCmds++
        delay(10)
        val c = cmd.trim()
        val u = c.uppercase()
        when {
            u == "PING" -> pub("PONG", framed = false)
            u == "INFO" -> pub("INFO Zitto_MB_V1_Bridge UART2=115200 BLE=ON RAW=1 FW=VCUMASTER", framed = false)
            u == "GPIO" -> listOf(
                "S32_GPIO_IDS=1..13", "1=PTD1 2=PTD0 3=PTE5 4=PTE4 5=PTE9",
                "6=PTE8 7=PTD5 8=PTC1 9=PTC15 10=PTC14", "11=PTB3 12=PTB1 13=PTB0",
            ).forEach { pub(it, framed = false) }
            u == "STATS" -> pub("STATS uart_bytes=${frames * 40} frames=$frames crc_errors=0 bad_len=0 ble=CONNECTED", framed = false)
            u.startsWith("ESP:") -> {
                val p = c.split(":")
                val pin = p.getOrNull(1)?.toIntOrNull()
                val st = p.getOrNull(2)?.toIntOrNull()
                if (p.size == 3 && pin != null && st != null) {
                    espGpio[pin] = st
                    pub("CMD_ACK ESP gpio=$pin state=$st", framed = false)
                } else pub("CMD_ERR format ESP:<pin>:<0|1>", framed = false)
            }
            u.startsWith("S32:") -> {
                val p = c.split(":").drop(1).map { it.toIntOrNull() }
                if (p.size == 3 && p.all { it != null }) {
                    gpioSet(p[0]!!, p[1]!!, p[2]!!)
                    pub("CMD_SENT S32 gpio=${p[0]} dir=${p[1]} state=${p[2]}", framed = false)
                } else pub("CMD_ERR format S32:<id>:<dir>:<state>", framed = false)
            }
            u.startsWith("RAW:") -> {
                val parsed = try { Protocol.parseRaw(c) } catch (e: Exception) { null }
                if (parsed == null) pub("CMD_ERR RAW hex invalid", framed = false)
                else raw(parsed.first, parsed.second)
            }
            else -> pub("CMD_ERR unknown command: $c", framed = false)
        }
    }

    private fun gpioSet(id: Int, d: Int, st: Int) {
        val g = gpio[id]
        if (g == null) {
            pub("LOG GPIO:invalid_id")
            return
        }
        g[0] = d
        g[1] = if (d != 0) st else 0
        pub("CMD_ACK cmd=0x2 result=0 gpio_id=$id state=${g[1]}")
        gpioStatus()
    }

    private fun raw(type: Int, pl: ByteArray) {
        val names = mapOf(0 to "IMU", 1 to "CSA", 2 to "CAN1", 3 to "CAN2", 4 to "FLM")
        val u = { i: Int -> pl[i].toInt() and 0xFF }
        when (type) {
            0x01 -> {
                if (pl.size < 2 || u(0) !in names) { pub("LOG CMD_MODULE_EN:bad_module"); return }
                mods[names.getValue(u(0))] = if (u(1) != 0) 1 else 0
                status()
            }
            0x02 -> if (pl.size >= 3) gpioSet(u(0), u(1), u(2))
            0x03 -> status()
            0x04 -> {
                pub("LOG RESET:armed")
                scope.launch {
                    delay(400)
                    t0 = now()
                    pub("LOG [BOOT] BOOT COMPLETE")
                    status()
                }
            }
            0x05 -> if (pl.size >= 4) pub("LOG LED:period=${u(0) or (u(1) shl 8)} duty=${u(2)}")
            0x06 -> {
                val last = flash.lastOrNull()
                if (last == null) pub("FLASH_DATA len=0 empty")
                else pub("FLASH_DATA len=${last.size} hex=${Protocol.hex(last)}")
            }
            0x07 -> { flash += pl; pub("LOG FLASH:OK"); flm() }
            0x08 -> {
                if (flash.isNotEmpty()) { flash.removeAt(flash.size - 1); pub("LOG FLASH:deleted") }
                else pub("LOG FLASH:FAIL")
                flm()
            }
            0x10 -> {
                if (pl.size < 8) { pub("LOG OTA:bad_len"); return }
                otaSize = (u(0) shl 24) or (u(1) shl 16) or (u(2) shl 8) or u(3)
                otaCrc = ((u(4).toLong() shl 24) or (u(5).toLong() shl 16) or (u(6).toLong() shl 8) or u(7).toLong())
                otaBuf = java.io.ByteArrayOutputStream()
                otaActive = true
                pub("LOG OTA:start_ok")
            }
            0x11 -> {
                if (!otaActive) { pub("LOG OTA:write_fail"); return }
                otaBuf.write(pl)
            }
            0x12 -> {
                val img = otaBuf.toByteArray()
                val ok = otaActive && img.size == otaSize && Protocol.crc32(img) == otaCrc
                otaActive = false
                otaPending = ok
                pub(if (ok) "LOG OTA:ok" else "LOG OTA:verify_fail")
            }
            0x13 -> { otaActive = false; pub("LOG OTA:aborted") }
            else -> pub("LOG CMD:unknown")
        }
    }

    override fun services() = listOf(
        GattService(
            NUS_SERVICE.toString(), "Nordic UART (simulated)",
            listOf(
                GattChar(NUS_RX.toString(), "RX (write)", listOf("write", "write-without-response")),
                GattChar(NUS_TX.toString(), "TX (notify)", listOf("read", "notify")),
            ),
        ),
    )
}
