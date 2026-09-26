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

/** Signals that report how many cells / temperature sensors the pack really has (Daly "No_Of_Battery_String"). */
private val COUNT_PAT = linkedMapOf(
    "bms.cell_count" to Regex("""(no|num|number)_?of_?(battery_?)?(string|cell|series)s?$|cell_?(count|num|qty)$|num_?cells?$|series_?(count|num)$""", RegexOption.IGNORE_CASE),
    "bms.temp_count" to Regex("""(no|num|number)_?of_?(temp|temperature|ntc)s?(_sensors?)?$|(temp|ntc)_?(count|num|qty)$""", RegexOption.IGNORE_CASE),
)

/** Fault / status flag of a Battery or Motor window: a bool or enum signal no role uses. */
data class FlagRef(val dbc: String, val msg: String, val sig: String, val kind: String, val choices: Map<Long, String>)

/** Everything the Battery / Motor windows need to lay themselves out (dbc_engine.roles_meta). */
data class RolesMeta(
    val version: Int = 0,
    val panels: List<String> = emptyList(),
    val roles: Map<String, Role> = emptyMap(),
    val cells: List<String> = emptyList(),
    val temps: List<String> = emptyList(),
    val balance: Map<Int, String> = emptyMap(),
    val flags: Map<String, List<FlagRef>> = emptyMap(),
    val dbcs: List<String> = emptyList(),
    val active: List<String> = emptyList(),
)

class DbcEngine {
    val dbcs = LinkedHashMap<String, LoadedDbc>()
    val signals = LinkedHashMap<String, SignalStat>()   // "bus:Msg.Sig"
    val messages = LinkedHashMap<String, MsgStat>()     // "bus:0xID"
    val vehicleMap = LinkedHashMap<String, String?>().apply { VEHICLE_ROLES.keys.forEach { put(it, null) } }
    private val mapOverrides = HashMap<String, String?>()

    // DBC-driven product roles (same analyzer as CAN_DBC_Simulator)
    val analyses = LinkedHashMap<String, Analysis>()
    private var roleIndex = HashMap<Triple<String, String, String>, MutableList<Pair<String, Binding>>>()
    val roleValues = HashMap<String, Double>()
    val roleT = HashMap<String, Double>()
    var rolesVersion = 0
        private set

    // frames no loaded DBC explains (library auto-match) and per-DBC activity
    private val unknown = HashMap<Triple<Int, Long, Boolean>, Double>()
    private val dbcLast = HashMap<String, Double>()
    private var active: List<String> = emptyList()

    fun load(name: String, text: String, buses: Collection<Int>): DbcDatabase {
        val db = DbcParser.parse(text)
        dbcs[name] = LoadedDbc(db, buses.toSet())
        automap()
        analyze(name)
        return db
    }

    fun remove(name: String) {
        dbcs.remove(name)
        dbcLast.remove(name)
        signals.entries.removeAll { it.value.dbc == name }
        automap()
        analyses.remove(name)
        rebuildRoleIndex()
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
            messages[mkey] = ms.copy(name = null, dbc = null)
            if (unknown.size < 4096) unknown[Triple(bus, id, ext)] = t
            return null
        }
        val (dbcName, msg) = hit
        unknown.remove(Triple(bus, id, ext))
        dbcLast[dbcName] = t
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
            roleIndex[Triple(dbcName, msg.name, s.name)]?.let { lst ->
                for ((roleKey, b) in lst) {
                    roleValues[roleKey] = b.toRole(num)
                    roleT[roleKey] = t
                }
            }
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
        roleValues.clear()
        roleT.clear()
    }

    // ------------------------------------------------------------ product roles
    private fun analyze(name: String) {
        val db = dbcs.getValue(name).db
        try {
            // The DBC's product is unknown here: analyze it as each product type, keep the reading with most roles.
            var best: Analysis? = null
            for (hint in listOf("BMS", "MCU", "VCU_Vehicle", "Charger")) {
                val a = Analyzer.analyze(db, hint)
                if (best == null || a.roles.size > best.roles.size) best = a
            }
            val a = best!!
            for ((key, pat) in COUNT_PAT) {
                if (key in a.roles) continue
                for (m in db.messages) {
                    val sg = m.signals.firstOrNull { pat.containsMatchIn(it.name) } ?: continue
                    a.roles[key] = Role(
                        key, if (key == "bms.cell_count") "Cells in pack" else "Temperature sensors", "", "number",
                        "battery", "count", bindings = mutableListOf(Binding(m.name, sg.name)), source = "${m.name}.${sg.name}",
                    )
                    break
                }
            }
            analyses[name] = a
        } catch (_: Exception) {
            analyses.remove(name)                 // never break DBC loading
        }
        rebuildRoleIndex()
    }

    private fun rebuildRoleIndex() {
        val idx = HashMap<Triple<String, String, String>, MutableList<Pair<String, Binding>>>()
        for ((name, a) in analyses) {
            for ((key, role) in a.roles) {
                for (b in role.bindings) idx.getOrPut(Triple(name, b.msg, b.sig)) { ArrayList() } += key to b
            }
        }
        roleIndex = idx
        rolesVersion++
    }

    /** {(id, ext): buses} of recent frames that no loaded DBC decodes. */
    fun unknownIds(now: Double, withinS: Double = 15.0): Map<Pair<Long, Boolean>, Set<Int>> {
        val cut = now - withinS
        val out = LinkedHashMap<Pair<Long, Boolean>, MutableSet<Int>>()
        for ((k, t) in unknown) if (t >= cut) out.getOrPut(k.second to k.third) { sortedSetOf() } += k.first
        return out
    }

    /** Re-order the Battery / Motor windows when a different DBC starts receiving frames. */
    fun refreshActive(now: Double, withinS: Double = 10.0) {
        val cut = now - withinS
        val act = dbcLast.filter { it.value >= cut && it.key in analyses }.keys.sorted()
        if (act != active) {
            active = act
            rolesVersion++
        }
    }

    /** Analyses of DBCs that are receiving frames first, so their layout wins. */
    private fun orderedAnalyses(): List<Pair<String, Analysis>> =
        analyses.entries.map { it.key to it.value }.sortedBy { it.first !in active }

    private fun primaryAnalysis(sel: (Analysis) -> Int): Analysis? {
        val pool = orderedAnalyses().filter { it.first in active }.ifEmpty { analyses.entries.map { it.key to it.value } }
        var best: Analysis? = null
        for ((_, a) in pool) if (best == null || sel(a) > sel(best)) best = a
        return best
    }

    fun rolesMeta(): RolesMeta {
        val roles = LinkedHashMap<String, Role>()
        val panels = ArrayList<String>()
        val flags = linkedMapOf<String, MutableList<FlagRef>>(
            "battery" to ArrayList(), "motor" to ArrayList(), "vehicle" to ArrayList(), "charger" to ArrayList(),
        )
        for ((name, a) in orderedAnalyses()) {
            if (active.isNotEmpty() && name !in active) continue      // windows follow the DBCs receiving frames
            for ((key, r) in a.roles) if (key !in roles) roles[key] = r
            for (p in a.panels) if (p !in panels) panels += p
            val db = dbcs[name]?.db ?: continue
            for (m in db.messages) {
                val list = flags[a.msgSystem[m.name]] ?: continue
                for (sg in m.signals) {
                    if ((m.name to sg.name) in a.sigRole || sg.muxSwitch) continue
                    val kind = Analyzer.signalKind(sg)
                    if (kind == "bool" || kind == "enum") list += FlagRef(name, m.name, sg.name, kind, LinkedHashMap(sg.choices))
                }
            }
        }
        val cells = primaryAnalysis { it.cells.size }
        val temps = primaryAnalysis { it.temps.size }
        return RolesMeta(
            version = rolesVersion,
            panels = panels,
            roles = roles,
            cells = cells?.cells ?: emptyList(),
            temps = temps?.temps ?: emptyList(),
            balance = cells?.balance ?: emptyMap(),
            flags = flags,
            dbcs = orderedAnalyses().map { it.first }.filter { active.isEmpty() || it in active },
            active = active,
        )
    }
}
