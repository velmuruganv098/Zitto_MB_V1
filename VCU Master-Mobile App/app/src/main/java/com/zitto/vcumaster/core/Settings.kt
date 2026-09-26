package com.zitto.vcumaster.core

import org.json.JSONArray
import org.json.JSONObject
import java.io.File

data class CustomFilter(val name: String, val expr: String, val regex: Boolean, val tag: String) {
    val re: Regex by lazy {
        try {
            if (regex) Regex(expr, RegexOption.IGNORE_CASE) else Regex(Regex.escape(expr), RegexOption.IGNORE_CASE)
        } catch (_: Exception) {
            Regex("(?!)")
        }
    }
}

/** Hub-side settings (filters, DBC files and bus assignment, vehicle map, last device). data/settings.json equivalent. */
class Settings(private val file: File) {
    val filters = mutableListOf<CustomFilter>()
    val vehicleMap = LinkedHashMap<String, String?>()
    val dbc = LinkedHashMap<String, List<Int>>()
    val dbcLib = LinkedHashMap<String, String>()          // loaded DBC name -> library id it came from
    var autoDbc = true                                     // load a matching library DBC when unknown frames arrive
    val autoDbcDeclined = LinkedHashSet<String>()          // library ids the user removed: never auto-load again
    var lastAddress: String? = null
    var lastName: String? = null

    init {
        load()
    }

    private fun load() {
        var j = JSONObject()
        if (file.exists()) {
            try { j = JSONObject(file.readText()) } catch (_: Exception) { }
        }
        val fa = j.optJSONArray("filters")
        if (fa == null) {
            filters += CustomFilter("CAN errors", "err=[1-9]|bus_off=1|ERROR", true, "")
            filters += CustomFilter("OTA + flash", "OTA|FLASH", true, "")
        } else {
            for (i in 0 until fa.length()) {
                val o = fa.getJSONObject(i)
                filters += CustomFilter(o.optString("name"), o.optString("expr"), o.optBoolean("regex", true), o.optString("tag", ""))
            }
        }
        j.optJSONObject("vehicle_map")?.let { o ->
            for (k in o.keys()) vehicleMap[k] = if (o.isNull(k)) null else o.getString(k)
        }
        j.optJSONObject("dbc")?.let { o ->
            for (k in o.keys()) {
                val b = o.getJSONObject(k).optJSONArray("buses") ?: JSONArray("[1,2]")
                dbc[k] = (0 until b.length()).map { b.getInt(it) }
                o.getJSONObject(k).optString("lib").ifEmpty { null }?.let { dbcLib[k] = it }
            }
        }
        autoDbc = j.optBoolean("auto_dbc", true)
        j.optJSONArray("auto_dbc_declined")?.let { a -> for (i in 0 until a.length()) autoDbcDeclined += a.getString(i) }
        j.optJSONObject("last_device")?.let {
            lastAddress = it.optString("address").ifEmpty { null }
            lastName = if (it.isNull("name")) null else it.optString("name")
        }
    }

    fun save() {
        val j = JSONObject()
        j.put("filters", JSONArray().apply {
            filters.forEach { put(JSONObject().put("name", it.name).put("expr", it.expr).put("regex", it.regex).put("tag", it.tag)) }
        })
        j.put("vehicle_map", JSONObject().apply { vehicleMap.forEach { (k, v) -> put(k, v ?: JSONObject.NULL) } })
        j.put("dbc", JSONObject().apply {
            dbc.forEach { (k, v) -> put(k, JSONObject().put("buses", JSONArray(v)).apply { dbcLib[k]?.let { put("lib", it) } }) }
        })
        j.put("auto_dbc", autoDbc)
        j.put("auto_dbc_declined", JSONArray(autoDbcDeclined.toList()))
        if (lastAddress != null) {
            j.put("last_device", JSONObject().put("address", lastAddress).put("name", lastName ?: JSONObject.NULL))
        }
        try {
            file.parentFile?.mkdirs()
            file.writeText(j.toString(2))
        } catch (_: Exception) { }
    }
}
