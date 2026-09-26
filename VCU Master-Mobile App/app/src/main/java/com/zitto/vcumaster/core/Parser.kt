package com.zitto.vcumaster.core

/**
 * Decodes the text lines published by the ESP32 bridge (one decoded ASCII line per UART frame)
 * back into structured records. Port of vcu_master/parser.py.
 */
class Parsed(
    val t: Double,
    val raw: String,
    val seq: Int?,
    var type: String,
    var tags: List<String>,
    val fields: LinkedHashMap<String, Any?>,
)

object Parser {
    private val SEQ_RE = Regex("""^seq=(\d+)\s+(\S+)\s*(.*)$""")
    private val KV_RE = Regex("""(\w+)=(\([^)]*\)|\[[^\]]*]|\S+)""")
    private val UNIT_RE = Regex("""^(-?\d+(?:\.\d+)?)(mA|mV|mW|ms|C)?$""")
    private val GPIO_RE = Regex("""#(\d+):(IN|OUT)=(\d)""")
    private val MAP_RE = Regex("""^\d+=PT""")

    /** src/CAN/can1.h, src/CAN/can2.h - the two differ, so decode by bus number. */
    val CAN1_STATES = mapOf(0L to "DETECTING", 1L to "READY", 2L to "ERROR")
    val CAN2_STATES = mapOf(0L to "OFF", 1L to "DETECTING", 2L to "LOCKED", 3L to "RUNNING", 4L to "ERROR")

    /** S32K1 RCM->SRS low byte (main.c truncates SRS to uint8_t). */
    private val RESET_BITS = linkedMapOf(1 to "LVD", 2 to "LOC", 3 to "LOL", 4 to "CMU_LOC", 5 to "WDOG", 6 to "PIN", 7 to "POR")

    /** '123mA' -> 123, '25.1C' -> 25.1, '0x1F' -> 31, else the string. */
    fun num(tok: String): Any {
        if (tok.lowercase().startsWith("0x")) return tok.substring(2).toLongOrNull(16) ?: tok
        val m = UNIT_RE.matchEntire(tok) ?: return tok
        val v = m.groupValues[1]
        return if ('.' in v) v.toDouble() else v.toLongOrNull() ?: tok
    }

    private fun tuple(tok: String): List<Long> =
        tok.trim('(', ')').split(",").mapNotNull { it.trim().toLongOrNull() }

    private fun dtuple(tok: String): List<Double> =
        tok.trim('(', ')').split(",").map { it.trim().toDouble() }

    fun decodeResetCause(v: Long): String {
        val names = RESET_BITS.filter { (b, _) -> v and (1L shl b) != 0L }.values
        return when {
            names.isNotEmpty() -> names.joinToString("+")
            v == 0L -> "NONE"
            else -> String.format("0x%02X", v)
        }
    }

    private fun kv(body: String): LinkedHashMap<String, String> {
        val out = LinkedHashMap<String, String>()
        for (m in KV_RE.findAll(body)) out[m.groupValues[1]] = m.groupValues[2]
        return out
    }

    private fun kvNum(body: String, f: LinkedHashMap<String, Any?>, keepHex: Boolean = false) {
        for ((k, v) in kv(body)) f[k] = if (keepHex && k == "hex") v else num(v)
    }

    fun parse(lineIn: String, now: Double = System.currentTimeMillis() / 1000.0): Parsed {
        val line = lineIn.trim()
        val f = LinkedHashMap<String, Any?>()
        val rec = Parsed(now, line, null, "OTHER", emptyList(), f)
        if (line.isEmpty()) return rec

        val m = SEQ_RE.matchEntire(line)
        val seq: Int?
        val mtype: String
        val body: String
        if (m != null) {
            seq = m.groupValues[1].toIntOrNull()
            mtype = m.groupValues[2]
            body = m.groupValues[3]
        } else {
            seq = null
            val parts = line.split(Regex("\\s+"), limit = 2)
            mtype = parts[0]
            body = if (parts.size > 1) parts[1] else ""
        }
        val r = Parsed(now, line, seq, mtype, emptyList(), f)

        try {
            when (mtype) {
                "IMU" -> {
                    val k = kv(body)
                    val a = tuple(k["accel_mg"] ?: "(0,0,0)")
                    val g = tuple(k["gyro_mdps"] ?: "(0,0,0)")
                    f["ax_mg"] = a[0]; f["ay_mg"] = a[1]; f["az_mg"] = a[2]
                    f["gx_mdps"] = g[0]; f["gy_mdps"] = g[1]; f["gz_mdps"] = g[2]
                    f["temp_c"] = num(k["temp"] ?: "0C")
                    f["ts_ms"] = num(k["ts"] ?: "0ms")
                    // V0.0073 firmware: displacement since power-on / tracking start
                    k["pos_mm"]?.let { pos ->
                        val p = dtuple(pos)
                        val rpy = dtuple(k["rpy_deg"] ?: "(0,0,0)")
                        f["pos_x_mm"] = p[0]; f["pos_y_mm"] = p[1]; f["pos_z_mm"] = p[2]
                        f["dist_mm"] = (k["dist_mm"] ?: "0").toDouble()
                        f["roll_fw"] = rpy[0]; f["pitch_fw"] = rpy[1]; f["yaw_fw"] = rpy[2]
                        f["moving"] = (k["moving"] ?: "0").toLong()
                        f["imu_flags"] = (k["imu_flags"] ?: "0").toLong()
                        f["imu_up_ms"] = (k["imu_up_ms"] ?: "0").toLong()
                        f["speed_mms"] = (k["speed_mms"] ?: "0").toLong()
                    }
                    r.tags = listOf("IMU")
                }
                "CSA" -> {
                    val k = kv(body)
                    f["current_ma"] = num(k["current"] ?: "0")
                    f["voltage_mv"] = num(k["voltage"] ?: "0")
                    f["power_mw"] = num(k["power"] ?: "0")
                    f["ts_ms"] = num(k["ts"] ?: "0ms")
                    r.tags = listOf("CSA")
                }
                "CAN" -> {
                    val k = kv(body)
                    val bus = (k["bus"] ?: "0").toInt()
                    val dataHex = (k["data"] ?: "[]").trim('[', ']').split(Regex("\\s+")).filter { it.isNotBlank() }
                    f["bus"] = bus.toLong()
                    f["id"] = num(k["id"] ?: "0x0")
                    f["ext"] = " EXT" in " $body"
                    f["rtr"] = " RTR" in " $body"
                    f["dlc"] = (k["dlc"] ?: "0").toLong()
                    f["data"] = dataHex.map { it.toInt(16) }
                    f["ts_ms"] = num(k["ts"] ?: "0ms")
                    r.tags = listOf("CAN$bus", "CAN")
                }
                "CAN_STATUS" -> {
                    kvNum(body, f)
                    val bus = f.long("bus")
                    val table = if (bus == 1L) CAN1_STATES else CAN2_STATES
                    val st = f["state"]
                    f["state_name"] = (st as? Long)?.let { table[it] } ?: st.toString()
                    r.tags = listOf("CAN$bus", "CAN_STATUS")
                }
                "STATUS" -> {
                    kvNum(body, f)
                    f["reset_name"] = decodeResetCause(f.long("reset"))
                    r.tags = listOf("STATUS", "SYSTEM")
                }
                "HEARTBEAT" -> { kvNum(body, f); r.tags = listOf("HEARTBEAT", "SYSTEM") }
                "FLM" -> { kvNum(body, f); r.tags = listOf("FLASH") }
                "GPIO_STATUS" -> {
                    f["pins"] = GPIO_RE.findAll(body).map {
                        GpioPin(it.groupValues[1].toInt(), it.groupValues[2], it.groupValues[3].toInt())
                    }.toList()
                    r.tags = listOf("GPIO")
                }
                "CMD_ACK" -> {
                    kvNum(body, f)
                    r.tags = if (body.startsWith("ESP")) listOf("CMD", "ESP32") else listOf("CMD")
                }
                "LOG" -> {
                    f["text"] = body
                    val up = body.uppercase()
                    val tags = mutableListOf("LOG")
                    // V0.0073 firmware events: "[CMD] ...", "[GPIO] ...", "[IMU] ..."
                    if (up.startsWith("[CMD]")) tags += "CMD"
                    if (up.startsWith("[GPIO]")) tags += "GPIO"
                    if (up.startsWith("FLASH")) tags += "FLASH"
                    if (up.startsWith("OTA")) tags += "OTA"
                    if (up.startsWith("GPIO")) tags += "GPIO"
                    if (up.startsWith("RESET") || up.startsWith("CMD")) tags += "CMD"
                    if ("CAN1" in up) tags += "CAN1"
                    if ("CAN2" in up) tags += "CAN2"
                    if ("IMU" in up) tags += "IMU"
                    if ("CSA" in up) tags += "CSA"
                    r.tags = tags
                }
                "RAW_RX" -> { kvNum(body, f, keepHex = true); r.tags = listOf("RAW") }
                "FLASH_DATA" -> {
                    // CMD_FLASH_RD response (MSG_FLASH_DATA 0x8A): "len=N empty" or "len=N hex=<bytes>"
                    kvNum(body, f, keepHex = true)
                    f["empty"] = "empty" in body.split(Regex("\\s+"))
                    r.tags = listOf("FLASH")
                }
                "BRIDGE_STATUS", "STATS" -> { kvNum(body, f); r.tags = listOf("ESP32", "BRIDGE") }
                "PONG", "INFO", "BLE_CONNECTED", "S32_GPIO_IDS" -> { f["text"] = body; r.tags = listOf("ESP32", "BRIDGE") }
                "CMD_SENT", "CMD_ERR" -> {
                    f["text"] = body
                    r.tags = if (mtype == "CMD_ERR") listOf("CMD", "ERR") else listOf("CMD")
                }
                else -> {
                    if (MAP_RE.containsMatchIn(line)) {
                        r.type = "GPIO_MAP"
                        r.tags = listOf("ESP32", "GPIO")
                    } else {
                        r.tags = listOf("OTHER")
                    }
                }
            }
        } catch (e: Exception) {
            r.tags = listOf("PARSE_ERR")
            f.clear()
            f["error"] = e.toString()
        }
        return r
    }
}

data class GpioPin(val id: Int, val dir: String, val state: Int)

// ---------------------------------------------------------------- field helpers
fun Map<String, Any?>.long(k: String, d: Long = 0): Long = when (val v = this[k]) {
    is Long -> v
    is Int -> v.toLong()
    is Double -> v.toLong()
    is Boolean -> if (v) 1 else 0
    is String -> v.toLongOrNull() ?: d
    else -> d
}

fun Map<String, Any?>.dbl(k: String, d: Double = 0.0): Double = when (val v = this[k]) {
    is Number -> v.toDouble()
    is String -> v.toDoubleOrNull() ?: d
    else -> d
}

fun Map<String, Any?>.str(k: String): String? = this[k]?.let {
    when (it) {
        is Double -> if (it == Math.floor(it) && kotlin.math.abs(it) < 1e15) it.toLong().toString() else it.toString()
        else -> it.toString()
    }
}

fun Map<String, Any?>.has(k: String) = this[k] != null
