package com.zitto.vcumaster.core

import org.json.JSONArray
import org.json.JSONObject

/** Channels: every place a data source appears uses the same colour (see ui/theme). */
val CH_ORDER = listOf("CAN1", "CAN2", "IMU", "CSA", "FLASH", "OTA", "GPIO", "SYSTEM", "ESP32", "CMD", "TX", "LOG", "RAW", "OTHER")
val CHIP_ORDER = listOf("IMU", "CSA", "FLASH", "CAN1", "CAN2", "SYSTEM", "GPIO", "OTA", "CMD", "ESP32", "TX", "LOG", "RAW", "OTHER")
val CH_LABEL = mapOf(
    "CAN1" to "CAN1", "CAN2" to "CAN2", "IMU" to "IMU", "CSA" to "CSA", "FLASH" to "Flash", "SYSTEM" to "System",
    "GPIO" to "GPIO", "OTA" to "OTA", "ESP32" to "ESP32", "CMD" to "Command", "TX" to "Sent", "LOG" to "Log",
    "RAW" to "Raw", "OTHER" to "Other",
)

fun channelOf(tags: List<String>) = CH_ORDER.firstOrNull { it in tags } ?: "OTHER"

private val ERR_RE = Regex("fail|bus_off=1|ERROR|CMD_ERR")

/** One message in the live stream (bridge line or sent command). Immutable once stored. */
class Rec(
    val id: Long,
    val t: Double,
    val raw: String,
    val seq: Int?,
    val type: String,
    val tags: List<String>,
    val fields: Map<String, Any?>,
    val dbc: DbcDecode? = null,
) {
    val ch: String = channelOf(tags)
    val isErr: Boolean by lazy { "ERR" in tags || "PARSE_ERR" in tags || ERR_RE.containsMatchIn(raw) }
    val decodeText: String by lazy { decodeTextOf(this) }
    val summary: String by lazy { summaryOf(this) }
    val searchText: String by lazy { "$raw $decodeText" }
    val canId: Long? get() = if (type == "CAN") fields.long("id") else null
}

private fun d3(v: Long) = "%.2f".format(java.util.Locale.US, v / 1000.0)

fun summaryOf(r: Rec): String {
    val f = r.fields
    return when (r.type) {
        "IMU" -> "a ${f.str("ax_mg")}, ${f.str("ay_mg")}, ${f.str("az_mg")} mg   " +
            "ω ${d3(f.long("gx_mdps"))}, ${d3(f.long("gy_mdps"))}, ${d3(f.long("gz_mdps"))} dps   T ${f.str("temp_c")} °C"
        "CSA" -> "I ${f.str("current_ma")} mA   V ${f.str("voltage_mv")} mV   P ${f.str("power_mw")} mW"
        "CAN" -> {
            @Suppress("UNCHECKED_CAST")
            val data = (f["data"] as? List<Int>).orEmpty()
            val ext = f["ext"] == true
            "id ${Fmt.hexId(f.long("id"), ext)} ${if (ext) "EXT" else "STD"}${if (f["rtr"] == true) " RTR" else ""}   " +
                "dlc ${f.str("dlc")}   ${data.joinToString(" ") { Fmt.hex2(it) }}"
        }
        "CAN_STATUS" -> "state ${f.str("state_name")}   baud ${f.str("baud")} k   ready ${f.str("ready")}   " +
            "bus_off ${f.str("bus_off")}   rx ${f.str("rx")}   err ${f.str("err")}   tec/rec ${f.str("tx_err")}/${f.str("rx_err")}"
        "STATUS" -> "mod imu ${f.str("imu")} csa ${f.str("csa")} can1 ${f.str("can1")} can2 ${f.str("can2")} flm ${f.str("flm")}   " +
            "up ${Fmt.dur(f.dbl("uptime"))}   reset ${f.str("reset_name")}   hb ${f.str("hb")}"
        "HEARTBEAT" -> "uptime ${Fmt.dur(f.dbl("uptime"))}"
        "FLM" -> "used ${f.str("used")}/${f.str("total")}   free ${f.str("free")}   records ${f.str("records")}"
        "GPIO_STATUS" -> {
            @Suppress("UNCHECKED_CAST")
            val pins = (f["pins"] as? List<GpioPin>).orEmpty()
            pins.joinToString(" ") { "${it.id}:${it.dir}=${it.state}" }
        }
        "CMD_ACK" -> f.entries.joinToString("   ") { "${it.key} ${f.str(it.key)}" }
        "FLASH_DATA" -> if (f["empty"] == true) "no record stored" else flashDataText(f.str("hex") ?: "")
        "LOG", "CMD_SENT", "CMD_ERR", "INFO", "PONG" -> (f["text"] as? String)?.ifEmpty { r.raw } ?: r.raw
        "TX", "TX_ERR" -> (f["cmd"] as? String) ?: r.raw
        else -> r.raw.replace(Regex("^seq=\\d+\\s+"), "").replaceFirst(r.type, "").trim()
    }
}

/** hex=… plus a printable-ASCII rendering when the record is text. */
fun flashDataText(hex: String): String {
    val bytes = try { Protocol.parseHex(hex) } catch (_: Exception) { return "hex=$hex" }
    val printable = bytes.all { it.toInt() in 0x20..0x7E }
    return "${bytes.size} B  $hex" + if (printable && bytes.isNotEmpty()) "  \"${String(bytes, Charsets.US_ASCII)}\"" else ""
}

fun decodeTextOf(r: Rec): String {
    val d = r.dbc ?: return ""
    if (d.error != null) return "decode error: ${d.error}"
    return d.message + "  " + d.signals.entries.joinToString("  ") { (k, v) ->
        "$k=${v.label ?: Fmt.num(v.v)}${if (v.unit.isNotEmpty()) " " + v.unit else ""}"
    }
}

private fun jsonOf(v: Any?): Any = when (v) {
    null -> JSONObject.NULL
    is Map<*, *> -> JSONObject().apply { v.forEach { (k, x) -> put(k.toString(), jsonOf(x)) } }
    is List<*> -> JSONArray().apply { v.forEach { put(jsonOf(it)) } }
    is GpioPin -> JSONObject().put("id", v.id).put("dir", v.dir).put("state", v.state)
    is SigVal -> JSONObject().put("v", v.v ?: JSONObject.NULL).put("label", v.label ?: JSONObject.NULL).put("unit", v.unit)
    is Double -> if (v.isNaN() || v.isInfinite()) v.toString() else v
    else -> v
}

fun recJson(r: Rec): String {
    val o = JSONObject()
    o.put("time", Fmt.clock(r.t))
    o.put("seq", r.seq ?: JSONObject.NULL)
    o.put("type", r.type)
    o.put("tags", JSONArray(r.tags))
    o.put("raw", r.raw)
    o.put("fields", jsonOf(r.fields))
    r.dbc?.let { d ->
        o.put("dbc", JSONObject().put("message", d.message).put("dbc", d.dbc ?: JSONObject.NULL)
            .put("signals", jsonOf(d.signals)).apply { d.error?.let { put("error", it) } })
    }
    return o.toString(2)
}

fun fieldsJson(f: Map<String, Any?>): String = jsonOf(f).toString()
fun dbcJson(d: DbcDecode?): String = if (d == null) "{}" else JSONObject()
    .put("message", d.message).put("signals", jsonOf(d.signals)).toString()
