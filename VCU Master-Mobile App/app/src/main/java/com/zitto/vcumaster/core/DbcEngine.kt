package com.zitto.vcumaster.core

/**
 * Decode CAN frames from the bridge against loaded DBC files. Port of dbc_engine.py.
 *  - One DBC can be assigned to CAN1, CAN2 or both.
 *  - Every decoded signal keeps last value, unit, min/max, update count and timestamp.
 *  - A vehicle map binds roles (speed, SOC, ...) to DBC signals; auto-filled by name heuristics.
 */
data class RoleMeta(val label: String, val unit: String, val max: Double?, val pat: Regex)

private fun rx(p: String) = Regex(p, RegexOption.IGNORE_CASE)

val VEHICLE_ROLES: LinkedHashMap<String, RoleMeta> = linkedMapOf(
    "speed" to RoleMeta("Vehicle speed", "km/h", 120.0, rx("""^(?!.*(motor|limit|rpm)).*(speed|spd)""")),
    "motor_rpm" to RoleMeta("Motor speed", "rpm", 8000.0, rx("""motor.*(rpm|speed)|rpm""")),
    "soc" to RoleMeta("State of charge", "%", 100.0, rx("""(^|_)soc($|_)|soc$|state_?of_?charge""")),
    "pack_voltage" to RoleMeta("Pack voltage", "V", 60.0, rx("""(pack|batt|battery).*(volt|_v\b)|pack_?v""")),
    "pack_current" to RoleMeta("Pack current", "A", 200.0, rx("""(pack|batt|battery).*(curr|_i\b)|pack_?i""")),
    "batt_temp" to RoleMeta("Battery temperature", "°C", 80.0, rx("""(cell|batt|battery|pack).*(temp|t_?max)""")),
    "motor_temp" to RoleMeta("Motor temperature", "°C", 150.0, rx("""motor.*temp""")),
    "ctrl_temp" to RoleMeta("Controller temperature", "°C", 120.0, rx("""(ctrl|controller|inverter|mcu).*temp""")),
    "throttle" to RoleMeta("Throttle", "%", 100.0, rx("""throttle|accel_?ped""")),
    "brake" to RoleMeta("Brake", "", 1.0, rx("""brake""")),
    "gear" to RoleMeta("Gear / drive mode", "", null, rx("""gear|drive_?mode""")),
    "odometer" to RoleMeta("Odometer", "km", null, rx("""odo""")),
    "fault" to RoleMeta("Active fault", "", null, rx("""fault|dtc|error_?code""")),
)

data class SigVal(val v: Double?, val label: String?, val unit: String)

data class DbcDecode(
    val message: String,
    val dbc: String?,
    val signals: Map<String, SigVal>,
    val error: String? = null,
)

data class SignalStat(
    val key: String, val bus: Int, val dbc: String, val message: String, val signal: String,
    val unit: String, val minSeen: Double?, val maxSeen: Double?, val count: Long,
    val dbcMin: Double?, val dbcMax: Double?, val value: Double?, val label: String?, val t: Double,
)

data class MsgStat(
    val bus: Int, val id: Long, val ext: Boolean, val count: Long, val first: Double, val last: Double,
    val rateHz: Double, val data: List<Int>, val name: String?, val dbc: String?,
)

data class VehEntry(
    val label: String, val unit: String, val max: Double?, val signal: String?,
    val value: Double?, val text: String?, val ageS: Double?,
)

data class DbcSummary(val name: String, val buses: List<Int>, val messages: Int, val signals: Int)

class LoadedDbc(val db: DbcDatabase, var buses: Set<Int>)

class DbcEngine {
    val dbcs = LinkedHashMap<String, LoadedDbc>()
    val signals = LinkedHashMap<String, SignalStat>()   // "bus:Msg.Sig"
    val messages = LinkedHashMap<String, MsgStat>()     // "bus:0xID"
    val vehicleMap = LinkedHashMap<String, String?>().apply { VEHICLE_ROLES.keys.forEach { put(it, null) } }
    private val mapOverrides = HashMap<String, String?>()

    fun load(name: String, text: String, buses: Collection<Int>): DbcDatabase {
        val db = DbcParser.parse(text)
        dbcs[name] = LoadedDbc(db, buses.toSet())
        automap()
        return db
    }

    fun remove(name: String) {
        dbcs.remove(name)
        signals.entries.removeAll { it.value.dbc == name }
        automap()
    }

    fun setBuses(name: String, buses: Collection<Int>) {
        dbcs[name]?.buses = buses.toSet()
    }

    fun list(): List<DbcSummary> = dbcs.map { (n, d) ->
        DbcSummary(n, d.buses.sorted(), d.db.messages.size, d.db.signalCount)
    }

    private fun find(bus: Int, id: Long, ext: Boolean): Pair<String, DbcMessage>? {
        for ((name, d) in dbcs) {
            if (bus !in d.buses) continue
            val m = d.db.byId[id] ?: continue
            if (m.ext != ext && id > 0x7FF) continue
            return name to m
        }
        return null
    }

    fun decode(bus: Int, id: Long, ext: Boolean, data: List<Int>, t: Double): DbcDecode? {
        val mkey = "$bus:0x${id.toString(16).uppercase()}"
        val prev = messages[mkey]
        var ms = prev ?: MsgStat(bus, id, ext, 0, t, t, 0.0, emptyList(), null, null)
        val dt = t - ms.last
        val count = ms.count + 1
        var rate = ms.rateHz
        if (count > 1 && dt > 0) {
            val inst = 1.0 / dt
            rate = if (rate == 0.0) inst else 0.8 * rate + 0.2 * inst
        }
        ms = ms.copy(count = count, rateHz = rate, last = t, data = data)

        val hit = find(bus, id, ext)
        if (hit == null) {
            messages[mkey] = ms
            return null
        }
        val (dbcName, msg) = hit
        messages[mkey] = ms.copy(name = msg.name, dbc = dbcName)

        val phys = try {
            msg.decode(data.toIntArray())
        } catch (e: Exception) {
            return DbcDecode(msg.name, dbcName, emptyMap(), e.toString())
        }
        val out = LinkedHashMap<String, SigVal>()
        for (s in msg.signals) {
            val num = phys[s.name] ?: continue
            val label = s.choices[num.toLong()]
            val key = "$bus:${msg.name}.${s.name}"
            val st = signals[key]
            signals[key] = if (st == null) {
                SignalStat(key, bus, dbcName, msg.name, s.name, s.unit, num, num, 1, s.min, s.max, num, label, t)
            } else {
                st.copy(
                    value = num, label = label, t = t, count = st.count + 1,
                    minSeen = minOf(st.minSeen ?: num, num), maxSeen = maxOf(st.maxSeen ?: num, num),
                )
            }
            out[s.name] = SigVal(num, label, s.unit)
        }
        return DbcDecode(msg.name, dbcName, out)
    }

    fun allSignalNames(): List<String> =
        dbcs.values.flatMap { d -> d.db.messages.flatMap { m -> m.signals.map { "${m.name}.${it.name}" } } }

    private fun automap() {
        val names = allSignalNames()
        for ((role, meta) in VEHICLE_ROLES) {
            if (mapOverrides.containsKey(role)) {
                vehicleMap[role] = mapOverrides[role]
                continue
            }
            vehicleMap[role] = names.firstOrNull { meta.pat.containsMatchIn(it.substringAfter('.')) }
        }
    }

    fun setMap(role: String, sig: String?) {
        require(role in VEHICLE_ROLES) { "Unknown role" }
        mapOverrides[role] = sig?.ifBlank { null }
        vehicleMap[role] = sig?.ifBlank { null }
    }

    fun vehicleSnapshot(now: Double): Map<String, VehEntry> {
        val snap = LinkedHashMap<String, VehEntry>()
        for ((role, sig) in vehicleMap) {
            val meta = VEHICLE_ROLES.getValue(role)
            var e = VehEntry(meta.label, meta.unit, meta.max, sig, null, null, null)
            if (sig != null) {
                val st = signals.values.filter { it.key.substringAfter(':') == sig }.maxByOrNull { it.t }
                if (st != null) {
                    e = e.copy(
                        value = st.value, text = st.label, unit = st.unit.ifEmpty { meta.unit },
                        ageS = now - st.t,
                    )
                }
            }
            snap[role] = e
        }
        return snap
    }

    fun resetStats() {
        signals.clear()
        messages.clear()
    }
}
