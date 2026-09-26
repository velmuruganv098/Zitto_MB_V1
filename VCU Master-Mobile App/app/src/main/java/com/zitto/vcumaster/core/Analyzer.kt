package com.zitto.vcumaster.core

import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow

/**
 * Semantic DBC analyzer. Port of vcu_master/analyzer.py (itself copied from
 * CAN_DBC_Simulator/simulator/analyzer.py) - keep the three in sync.
 *
 * Looks at every message/signal of a DBC and maps signals to *roles*
 * (bms.cell_v.7, bms.soc, mcu.rpm, veh.speed, chg.set_current ...). A role can be bound to several
 * signals (SOC broadcast in 3 messages) and every binding carries a unit conversion so the UI works
 * in canonical units (V, A, degC, %, km/h). Works purely on names, units, sizes and value tables.
 */
private typealias MS = Pair<DbcMessage, DbcSignal>

object Analyzer {
    // ----------------------------------------------------------------- tokenizing
    private val TOK = Regex("""[A-Z]+(?![a-z])|[A-Z]?[a-z]+|\d+""")
    private val ACRO = Regex("""(MOS|FET|SOC|SOH|BMS|MCU|VCU|OBC|DC|AC|DTE|ODO|RPM|NTC|AFE)(?=[a-z])""")

    private val COMPOUND = mapOf(
        "cmin" to listOf("cell", "min"), "cmax" to listOf("cell", "max"), "cavg" to listOf("cell", "avg"),
        "vstack" to listOf("stack", "voltage"), "vpack" to listOf("pack", "voltage"), "vbat" to listOf("battery", "voltage"),
        "vbatt" to listOf("battery", "voltage"), "ipack" to listOf("pack", "current"), "ibat" to listOf("battery", "current"),
        "ibatt" to listOf("battery", "current"), "vmin" to listOf("cell", "min", "voltage"),
        "vmax" to listOf("cell", "max", "voltage"), "tmin" to listOf("min", "temp"), "tmax" to listOf("max", "temp"),
        "tavg" to listOf("avg", "temp"), "batterycurrent" to listOf("battery", "current"),
        "batteryvoltage" to listOf("battery", "voltage"), "packvoltage" to listOf("pack", "voltage"),
    )

    fun tokens(name: String): List<String> {
        val out = ArrayList<String>()
        for (m in TOK.findAll(ACRO.replace(name, "_"))) {
            val t = m.value.lowercase()
            val c = COMPOUND[t]
            when {
                c != null -> out.addAll(c)
                t.startsWith("cell") && t.length > 4 && !t.startsWith("cells") -> { out += "cell"; out += t.substring(4) }
                t.startsWith("vcell") && t.length > 5 -> { out += "vcell"; out += t.substring(5) }
                else -> out += t
            }
        }
        return out
    }

    private val UNIT_TABLE = mapOf(
        "v" to "V", "volt" to "V", "volts" to "V", "voltage" to "V", "mv" to "mV", "millivolt" to "mV", "cv" to "cV",
        "centivolt" to "cV", "dv" to "dV", "a" to "A", "amp" to "A", "amps" to "A", "ampere" to "A", "ma" to "mA",
        "mamps" to "mA", "ca" to "cA", "da" to "dA", "degc" to "degC", "c" to "degC", "deg" to "degC", "degreec" to "degC",
        "celsius" to "degC", "degreecelsius" to "degC", "k" to "K", "kelvin" to "K", "%" to "%", "percent" to "%",
        "pct" to "%", "rpm" to "rpm", "1/min" to "rpm", "km/h" to "km/h", "kmph" to "km/h", "kph" to "km/h",
        "m/s" to "m/s", "mph" to "mph", "nm" to "Nm", "w" to "W", "kw" to "kW", "ah" to "Ah", "mah" to "mAh",
        "km" to "km", "m" to "m", "s" to "s", "ms" to "ms", "wh" to "Wh", "kwh" to "kWh",
    )

    fun normUnit(uIn: String?): String {
        val u = (uIn ?: "").trim().lowercase().replace("°", "deg").replace("º", "deg").replace(" ", "")
        if (u.startsWith("deg") && u.length > 3 && u.substring(3) in setOf("c", "reec")) return "degC"
        return UNIT_TABLE[u] ?: u
    }

    /** canonical unit per dimension and conversion (factor, offset): sig = can*f + o */
    private val CONV = mapOf(
        ("V" to "V") to (1.0 to 0.0), ("V" to "mV") to (1000.0 to 0.0), ("V" to "cV") to (100.0 to 0.0), ("V" to "dV") to (10.0 to 0.0),
        ("A" to "A") to (1.0 to 0.0), ("A" to "mA") to (1000.0 to 0.0), ("A" to "cA") to (100.0 to 0.0), ("A" to "dA") to (10.0 to 0.0),
        ("degC" to "degC") to (1.0 to 0.0), ("degC" to "K") to (1.0 to 273.15),
        ("km/h" to "km/h") to (1.0 to 0.0), ("km/h" to "m/s") to (1 / 3.6 to 0.0), ("km/h" to "mph") to (0.621371 to 0.0),
        ("kW" to "kW") to (1.0 to 0.0), ("kW" to "W") to (1000.0 to 0.0), ("Ah" to "Ah") to (1.0 to 0.0), ("Ah" to "mAh") to (1000.0 to 0.0),
        ("km" to "km") to (1.0 to 0.0), ("km" to "m") to (1000.0 to 0.0),
    )

    private fun s(vararg x: String) = x.toSet()

    private val VOLT = s("v", "volt", "volts", "voltage", "voltg", "vtg", "vol", "cellv", "vcell", "cv", "vltg", "voltages")
    private val TEMP = s("temp", "temperature", "tempr", "ntc", "th", "therm", "thermistor", "temps", "temperatures")
    private val CURR = s("current", "curr", "cur", "currnet", "i")
    private val MAXW = s("max", "maximum", "highest", "high", "hi", "upper")
    private val MINW = s("min", "minimum", "lowest", "low", "lo", "lower")
    private val IDXW = s("noof", "no", "num", "number", "idx", "index", "id", "nub", "pos", "position", "mod", "module", "location")
    private val STATW = s("avg", "average", "mean", "delta", "diff", "dev", "deviation", "imbalance", "imb", "spread")
    private val NOTVAL = s(
        "limit", "lmt", "threshold", "thr", "set", "ref", "hyst", "hysterisis", "hysteresis", "cutoff", "cut",
        "warn", "warning", "fault", "flt", "err", "error", "alarm", "protect", "protection", "status", "sts",
        "flag", "cfail", "fail", "failure", "detect", "detection", "cause", "validity", "valid", "ok", "adc",
        "raw", "offset", "calib", "calibration", "demand", "dmd", "request", "req", "target", "cmd", "level",
        "cnt", "count", "counts", "sensor", "supply", "plaus", "plausibility", "check", "mismatch", "peak",
        "open", "short", "enable", "en", "sel", "debug", "saturated", "trip", "rating", "rated", "nominal",
        "config", "cfg", "setting", "timeout", "time", "code", "reg", "register", "mask", "pwm", "ota",
        "chglmt", "dsglmt", "avail", "available", "allowed", "allow", "capability", "min", "max",
    )
    private val BAL = s("bal", "balance", "balancing", "balanced", "balancer", "cellbalancing")
    private val TEMP_ELSEWHERE = s(
        "fet", "mos", "mosfet", "pcb", "board", "ambient", "amb", "shunt", "busbar", "contactor", "gun",
        "contact", "precharge", "pchg", "motor", "controller", "mcu", "igbt", "heatsink", "water",
        "coolant", "chiller", "heater", "tank", "llc", "pfc", "cs", "ic", "inlet", "outlet", "air", "oil",
        "exhaust", "cat", "intake", "engine", "dtu", "ctrl", "pwr", "onboard", "room", "liquid",
        "condenser", "compressor", "resistor", "pump", "obc", "charger", "cabin", "internal", "aft",
        "hfe", "lfe", "fuse", "connector", "terminal", "relay",
    )
    private val SYS_WORDS = linkedMapOf(
        "charger" to s("chg", "charger", "charging", "obc", "gbt", "station", "rectifier", "evse", "ccs", "stecom",
            "dyna", "amar", "gun", "chademo", "ci", "slave", "chrgngpw", "chrgngtime"),
        "motor" to s("mcu", "motor", "mc", "inv", "inverter", "dtu", "foc", "throttle", "mvcu", "controller", "drive",
            "traction", "pmsm", "bldc", "eec", "engine"),
        "battery" to s("bms", "bmsp", "bmsf", "batt", "battery", "cell", "cells", "pack", "soc", "bmu", "afe", "ntc",
            "b2v", "bp", "bf", "string", "str", "balance", "balancing", "packa", "packb", "packc", "bat", "mos",
            "discharge", "remain", "capacity", "soh", "bmu"),
        "vehicle" to s("vcu", "veh", "vehicle", "ipc", "cluster", "dash", "hmi", "display", "bcm", "abs", "tcu", "tel",
            "viu", "bcu", "pke", "esl", "gps", "odo", "odometer", "speed", "light", "lights", "key", "diu",
            "ccvs", "etc", "tco", "lfe", "hrw", "body", "lamp", "indicator", "horn", "trip", "ignition"),
    )
    private val NODE_SYS = mapOf(
        "bms" to "battery", "bmu" to "battery", "battery" to "battery", "bms_portable_battery" to "battery",
        "bms_fixe" to "battery", "mcu" to "motor", "motor_controller" to "motor", "motorcontroller" to "motor",
        "dtu" to "motor", "mc" to "motor", "inverter" to "motor", "vcu" to "vehicle", "ipc" to "vehicle",
        "diu" to "vehicle", "display" to "vehicle", "bcm" to "vehicle", "tel" to "vehicle", "tcu" to "vehicle",
        "abs" to "vehicle", "viu" to "vehicle", "bcu" to "vehicle", "pke" to "vehicle", "esl" to "vehicle",
        "obc" to "charger", "charger" to "charger",
    )
    private val FOLDER_SYS = mapOf(
        "BMS" to "battery", "MCU" to "motor", "VCU_Vehicle" to "vehicle", "Charger" to "charger",
        "J1939_ICE" to "vehicle", "OBD2" to "vehicle", "Test" to "vehicle",
    )

    val PANEL_ORDER = listOf("battery", "motor", "vehicle", "charger")
    val PANEL_LABEL = mapOf("battery" to "Battery (BMS)", "motor" to "Motor (MCU)", "vehicle" to "Vehicle / VCU", "charger" to "Charger")

    /** role key -> (label, canonical unit, kind, panel, section) */
    private class Meta(val label: String, val unit: String, val kind: String, val panel: String, val section: String)

    private val ROLE_META: Map<String, Meta> = mapOf(
        "bms.pack_voltage" to Meta("Pack voltage", "V", "number", "battery", "kpi"),
        "bms.current" to Meta("Pack current", "A", "number", "battery", "kpi"),
        "bms.soc" to Meta("SOC", "%", "number", "battery", "kpi"),
        "bms.soh" to Meta("SOH", "%", "number", "battery", "kpi"),
        "bms.power" to Meta("Pack power", "kW", "number", "battery", "kpi"),
        "bms.remaining_cap" to Meta("Remaining capacity", "Ah", "number", "battery", "kpi"),
        "bms.full_cap" to Meta("Full capacity", "Ah", "number", "battery", "kpi"),
        "bms.cycles" to Meta("Cycle count", "", "number", "battery", "kpi"),
        "bms.max_cell_v" to Meta("Max cell voltage", "V", "number", "battery", "stats"),
        "bms.min_cell_v" to Meta("Min cell voltage", "V", "number", "battery", "stats"),
        "bms.avg_cell_v" to Meta("Avg cell voltage", "V", "number", "battery", "stats"),
        "bms.delta_cell_v" to Meta("Cell voltage delta", "V", "number", "battery", "stats"),
        "bms.max_cell_v_id" to Meta("Max cell #", "", "number", "battery", "stats"),
        "bms.min_cell_v_id" to Meta("Min cell #", "", "number", "battery", "stats"),
        "bms.max_temp" to Meta("Max temperature", "degC", "number", "battery", "stats"),
        "bms.min_temp" to Meta("Min temperature", "degC", "number", "battery", "stats"),
        "bms.avg_temp" to Meta("Avg temperature", "degC", "number", "battery", "stats"),
        "bms.delta_temp" to Meta("Temperature delta", "degC", "number", "battery", "stats"),
        "bms.max_temp_id" to Meta("Max temp sensor #", "", "number", "battery", "stats"),
        "bms.min_temp_id" to Meta("Min temp sensor #", "", "number", "battery", "stats"),
        "bms.charge_mos" to Meta("Charge MOSFET", "", "bool", "battery", "switch"),
        "bms.discharge_mos" to Meta("Discharge MOSFET", "", "bool", "battery", "switch"),
        "bms.chg_limit" to Meta("Charge current limit", "A", "number", "battery", "limits"),
        "bms.dsg_limit" to Meta("Discharge current limit", "A", "number", "battery", "limits"),
        "mcu.rpm" to Meta("Motor speed", "rpm", "number", "motor", "gauge"),
        "mcu.torque" to Meta("Torque", "Nm", "number", "motor", "gauge"),
        "mcu.throttle" to Meta("Throttle", "%", "number", "motor", "input"),
        "mcu.brake" to Meta("Brake", "", "number", "motor", "input"),
        "mcu.dc_voltage" to Meta("DC bus voltage", "V", "number", "motor", "power"),
        "mcu.dc_current" to Meta("DC bus current", "A", "number", "motor", "power"),
        "mcu.phase_current" to Meta("Phase current", "A", "number", "motor", "power"),
        "mcu.motor_temp" to Meta("Motor temperature", "degC", "number", "motor", "thermal"),
        "mcu.ctrl_temp" to Meta("Controller temperature", "degC", "number", "motor", "thermal"),
        "mcu.gear" to Meta("Gear / direction", "", "enum", "motor", "input"),
        "mcu.mode" to Meta("Drive mode", "", "enum", "motor", "input"),
        "veh.speed" to Meta("Vehicle speed", "km/h", "number", "vehicle", "gauge"),
        "veh.odometer" to Meta("Odometer", "km", "number", "vehicle", "trip"),
        "veh.trip" to Meta("Trip", "km", "number", "vehicle", "trip"),
        "veh.range" to Meta("Range / DTE", "km", "number", "vehicle", "trip"),
        "veh.ignition" to Meta("Ignition / key", "", "bool", "vehicle", "switch"),
        "veh.side_stand" to Meta("Side stand", "", "bool", "vehicle", "switch"),
        "veh.lat" to Meta("Latitude", "deg", "number", "vehicle", "gps"),
        "veh.lon" to Meta("Longitude", "deg", "number", "vehicle", "gps"),
        "chg.out_voltage" to Meta("Output voltage", "V", "number", "charger", "output"),
        "chg.out_current" to Meta("Output current", "A", "number", "charger", "output"),
        "chg.set_voltage" to Meta("Voltage request / set", "V", "number", "charger", "setpoint"),
        "chg.set_current" to Meta("Current request / set", "A", "number", "charger", "setpoint"),
        "chg.in_voltage" to Meta("Input (AC) voltage", "V", "number", "charger", "input"),
    )

    // ---------------------------------------------------------------- utilities
    private fun p2(n: Int) = 2.0.pow(n)

    fun sigRange(sig: DbcSignal): Pair<Double, Double> {
        val lo = sig.min
        val hi = sig.max
        val scale = if (sig.scale == 0.0) 1.0 else sig.scale
        val off = sig.offset
        val (rlo, rhi) = if (sig.signed) -p2(sig.length - 1) to p2(sig.length - 1) - 1 else 0.0 to p2(sig.length) - 1
        val a = rlo * scale + off
        val b = rhi * scale + off
        val plo = min(a, b)
        val phi = max(a, b)
        if (lo == null || hi == null || lo == hi || lo > hi) return plo to phi
        if (sig.length >= 4 && (hi - lo) * 100 < (phi - plo) && (hi - lo) <= 1) return plo to phi
        return max(lo, plo) to min(hi, phi)
    }

    private fun sigStep(sig: DbcSignal): Double {
        val s = abs(if (sig.scale == 0.0) 1.0 else sig.scale)
        return if (s < 1) s else 1.0
    }

    private val MSG_BAT_RE = Regex("""(^|_)(b|bms|pack|batt|bat)_?\d+(_|$)""")

    fun msgSystem(msg: DbcMessage, default: String): String {
        for (node in msg.senders) NODE_SYS[node.lowercase()]?.let { return it }
        val toks = tokens(msg.name).toSet()
        val low = msg.name.lowercase()
        if (MSG_BAT_RE.containsMatchIn(low)) return "battery"
        for (sys in listOf("charger", "motor", "battery", "vehicle")) {
            if (toks.any { it in SYS_WORDS.getValue(sys) }) return sys
        }
        if (low.startsWith("bms") || low.startsWith("batt") || low.startsWith("cell")) return "battery"
        if (low.startsWith("mcu") || low.startsWith("mc_") || low.startsWith("motor") || low.startsWith("inv")) return "motor"
        return default
    }

    fun conversion(roleUnit: String, sig: DbcSignal, isCell: Boolean = false): Pair<Double, Double> {
        val su = normUnit(sig.unit)
        CONV[roleUnit to su]?.let { return it }
        val (lo, hi) = sigRange(sig)
        val scale = if (sig.scale == 0.0) 1.0 else sig.scale
        if (roleUnit == "V" && su == "") {
            if ((isCell && hi > 20) || (!isCell && hi > 20000 && scale >= 1)) return 1000.0 to 0.0
        }
        if (roleUnit == "A" && su == "" && hi > 20000 && scale >= 1) return 1000.0 to 0.0
        if (roleUnit == "%" && hi <= 1.0 && scale < 0.1) return 0.01 to 0.0
        if (roleUnit == "degC" && su == "" && lo >= 200) return 1.0 to 273.15
        return 1.0 to 0.0
    }

    // ------------------------------------------------------------ matching rules
    private fun isBool(sig: DbcSignal) = sig.length == 1 || (sig.choices.size == 2 && sig.length <= 2)

    /** Value table that only names special raw codes (NO_VALUE / ERROR / SNA ...) of a physical signal. */
    private fun specialValuesOnly(sig: DbcSignal): Boolean {
        if (sig.choices.isEmpty() || sig.length < 6) return false
        val top = p2(sig.length) - 1
        val keys = sig.choices.keys
        return keys.all { it >= top * 0.9 } || (keys.size <= 3 && keys.all { it >= top - 0x200 || it >= 0xF0 })
    }

    /** 'bool' | 'enum' | 'number' */
    fun signalKind(sig: DbcSignal): String {
        if (sig.length == 1) return "bool"
        if (sig.choices.isNotEmpty() && !specialValuesOnly(sig)) {
            if (sig.choices.size == 2 && sig.length <= 2) return "bool"
            if (sig.choices.size > 1) return "enum"
        }
        return "number"
    }

    private val NUMERIC_FAMILIES = s(
        "bms.cell_v", "bms.temp", "bms.pack_voltage", "bms.current", "bms.soc", "bms.soh", "bms.power",
        "bms.remaining_cap", "bms.full_cap", "bms.cycles", "bms.max_cell_v", "bms.min_cell_v",
        "bms.avg_cell_v", "bms.delta_cell_v", "bms.max_temp", "bms.min_temp", "bms.avg_temp",
        "bms.delta_temp", "bms.chg_limit", "bms.dsg_limit", "mcu.rpm", "mcu.torque", "mcu.throttle",
        "mcu.dc_voltage", "mcu.dc_current", "mcu.phase_current", "mcu.motor_temp", "mcu.ctrl_temp",
        "veh.speed", "veh.odometer", "veh.trip", "veh.range", "veh.lat", "veh.lon", "chg.out_voltage",
        "chg.out_current", "chg.set_voltage", "chg.set_current", "chg.in_voltage",
    )

    /** 1-bit, 2-state value table, or a byte the DBC declares as [0|1]. */
    private fun flagish(sig: DbcSignal) = isBool(sig) || sig.choices.size == 2 ||
        (sig.length <= 8 && sig.min == 0.0 && sig.max == 1.0 && (if (sig.scale == 0.0) 1.0 else sig.scale) == 1.0)

    private fun Set<String>.hits(o: Set<String>) = any { it in o }

    private val T_RE = Regex("""^t\d+$""")
    private val LOWHIGH_RE = Regex("""(_|^)(low|high|lsb|msb|lo|hi)$""")
    private val COUNTER_RE = Regex("""(alive|rolling|live)_?(cnt|count|counter)|msg_?cnt|_rc$|^rc$""")

    private val NOTVAL_NO_SENSOR = NOTVAL - "sensor"
    private val NOTVAL_NO_TRIP = NOTVAL - "trip"
    private val SET_WORDS = s("set", "demand", "dmd", "request", "req", "target", "ref", "max", "limit", "requested")

    /** (role family, key) or null. Families with keys are multi-instance (cells / temps / balance). */
    fun classifySignal(sig: DbcSignal, msg: DbcMessage, ctx: String): Pair<String, List<Any>>? {
        val name = sig.name
        val st = tokens(name)
        val ss = st.toSet()
        val mt = tokens(msg.name).toSet()
        val u = normUnit(sig.unit)
        val nums: List<Long> = st.filter { t -> t.all { it.isDigit() } }.map { it.toLongOrNull() ?: Long.MAX_VALUE }
        val isB = isBool(sig)
        val low = name.lowercase()
        val hasVolt = ss.hits(VOLT) || u in s("V", "mV", "cV", "dV")
        val hasTemp = ss.hits(TEMP) || u in s("degC", "K") || T_RE.matches(low)
        val hasCurr = ss.hits(CURR) || u in s("A", "mA", "cA", "dA")
        val cellish = "cell" in ss || "cells" in ss || "vcell" in ss || "cellv" in ss || "cellvol" in low
        val notval = ss.hits(NOTVAL)
        val lowhighSplit = LOWHIGH_RE.containsMatchIn(low) && !ss.hits(TEMP + VOLT)
        val none = emptyList<Any>()

        // ---------------- auto-increment helpers (not roles)
        if (ss.hits(s("counter", "alive", "rolling", "livecounter", "rollingcounter", "msgcnt")) || COUNTER_RE.containsMatchIn(low)) {
            return "auto.counter" to none
        }
        if (ss.hits(s("crc", "checksum", "chksum", "cs")) && sig.length >= 4 && !hasTemp) return "auto.checksum" to none

        // ---------------- battery: short names inside a "cell" message (C1, V_0, T_3, CB_2)
        val isDigitTok = { t: String -> t.isNotEmpty() && t.all { it.isDigit() } }
        if (st.size == 2 && isDigitTok(st[1]) && ("cell" in mt || "cells" in mt) && !isB) {
            val head = st[0]
            val n = st[1].toLongOrNull() ?: Long.MAX_VALUE
            if (head in s("c", "v", "cv", "vc", "u", "cell") && (mt.hits(VOLT) || u in s("V", "mV")) && !mt.hits(TEMP)) {
                return "bms.cell_v" to listOf(n)
            }
            if (head in s("t", "th", "temp", "ntc") && (mt.hits(TEMP) || u in s("degC", "K"))) return "bms.temp" to listOf(n)
        }
        if (st.size == 2 && isDigitTok(st[1]) && st[0] in s("cb", "bal", "b") && (mt.hits(BAL) || "balance" in msg.name.lowercase())) {
            return "bms.balance" to listOf(st[1].toLongOrNull() ?: Long.MAX_VALUE)
        }

        // ---------------- battery: per-cell
        if (cellish && nums.isNotEmpty() && !hasTemp && !ss.hits(MAXW + MINW + STATW + IDXW) &&
            !ss.hits(BAL) && !ss.hits(s("soc", "soh", "capacity", "cap", "ah", "resistance", "ir")) &&
            !ss.hits(NOTVAL_NO_SENSOR)
        ) {
            val voltCtx = hasVolt || ((mt.hits(VOLT) || "cv" in mt) && !mt.hits(TEMP)) ||
                ("cell" in ss && st.size <= 3 && ctx == "battery" && !mt.hits(TEMP) && !mt.hits(BAL))
            if (voltCtx && !isB) return "bms.cell_v" to cellKey(st, nums)
        }
        if (cellish && nums.isNotEmpty() && (ss.hits(BAL) || (mt.hits(BAL) && isB)) && !hasTemp) {
            return "bms.balance" to cellKey(st, nums)
        }
        if (nums.isNotEmpty() && hasTemp && !isB && !ss.hits(MAXW + MINW + STATW + IDXW) &&
            !ss.hits(TEMP_ELSEWHERE) && !ss.hits(NOTVAL) &&
            (ctx == "battery" || cellish || ss.hits(s("ntc", "th", "battery", "bat", "batt", "pack")))
        ) return "bms.temp" to cellKey(st, nums)
        if (cellish && nums.isNotEmpty() && !hasVolt && !hasTemp && !isB &&
            mt.hits(TEMP) && !ss.hits(MAXW + MINW + STATW + IDXW + NOTVAL)
        ) return "bms.temp" to cellKey(st, nums)

        // ---------------- battery: aggregates
        if ((cellish || ctx == "battery") && !isB) {
            if (hasTemp && !ss.hits(TEMP_ELSEWHERE)) {
                if (ss.hits(MAXW)) return (if (ss.hits(IDXW)) "bms.max_temp_id" else "bms.max_temp") to none
                if (ss.hits(MINW)) return (if (ss.hits(IDXW)) "bms.min_temp_id" else "bms.min_temp") to none
                if (ss.hits(s("avg", "average", "mean"))) return "bms.avg_temp" to none
                if (ss.hits(s("delta", "diff", "imbalance", "imb", "dev", "spread"))) return "bms.delta_temp" to none
            }
            if ((hasVolt || cellish) && !hasTemp && !hasCurr && !ss.hits(s("soc", "soh", "capacity", "pack", "total", "string"))) {
                if (ss.hits(MAXW) && nums.isEmpty()) return (if (ss.hits(IDXW)) "bms.max_cell_v_id" else "bms.max_cell_v") to none
                if (ss.hits(MINW) && nums.isEmpty()) return (if (ss.hits(IDXW)) "bms.min_cell_v_id" else "bms.min_cell_v") to none
                if (ss.hits(s("avg", "average", "mean")) && nums.isEmpty()) return "bms.avg_cell_v" to none
                if (ss.hits(s("delta", "diff", "imbalance", "imb", "dev")) && nums.isEmpty()) return "bms.delta_cell_v" to none
            }
        }
        if ("soc" in ss && !isB && nums.isEmpty() &&
            !ss.hits(s("pwm", "en", "sel", "period", "scale", "debug", "cfail", "cause", "threshold", "warning",
                "warn", "limit", "min", "max", "low", "high", "error", "fault", "ota", "pre", "cell",
                "cells", "minthreshold"))
        ) return "bms.soc" to none
        if ("soh" in ss && !isB && nums.isEmpty() && !ss.hits(NOTVAL + "cell")) return "bms.soh" to none
        if (ss.hits(s("cycle", "cycles")) && !isB && !ss.hits(s("duty", "time")) && nums.isEmpty()) return "bms.cycles" to none
        if (ss.hits(s("remaining", "remain", "rem", "residual", "left")) && (ss.hits(s("capacity", "cap", "ah")) || u in s("Ah", "mAh"))) {
            return "bms.remaining_cap" to none
        }
        if (ss.hits(s("full", "design", "nominal", "total")) && ss.hits(s("capacity", "cap")) && nums.isEmpty()) return "bms.full_cap" to none

        // ---------------- switches
        if (ss.hits(s("mos", "fet", "mosfet", "contactor", "relay")) && flagish(sig) &&
            !ss.hits(NOTVAL + s("temp", "err", "open", "circuit", "adhesion", "weld", "welded", "stuck", "alarm"))
        ) {
            if (ss.hits(s("chg", "charge", "charging", "ch", "c"))) return "bms.charge_mos" to none
            if (ss.hits(s("dsg", "dischg", "discharge", "discharging", "dis", "dch", "d"))) return "bms.discharge_mos" to none
        }

        // ---------------- charger
        val chgSig = ss.hits(s("charger", "chg", "obc", "ccs", "gbt")) || ctx == "charger"
        if (chgSig && !isB && nums.isEmpty()) {
            if (hasVolt && !ss.hits(s("cell", "phase", "em", "pfc", "sensor", "supply"))) {
                if (ss.hits(s("input", "in", "ac", "mains", "grid", "yn", "rn", "bn"))) return "chg.in_voltage" to none
                if (ss.hits(SET_WORDS)) return "chg.set_voltage" to none
                if (!notval) return "chg.out_voltage" to none
            }
            if (hasCurr && !ss.hits(s("phase", "em", "pfc", "sensor", "peak", "input", "in", "ac"))) {
                if (ss.hits(SET_WORDS)) return "chg.set_current" to none
                if (!notval) return "chg.out_current" to none
            }
        }

        // ---------------- motor
        if (!isB && !lowhighSplit) {
            if (("rpm" in ss || u == "rpm" || (ss.hits(s("motor", "engine")) && "speed" in ss)) &&
                !ss.hits(NOTVAL + s("validity", "set", "cruise"))
            ) return "mcu.rpm" to none
            if (ss.hits(s("torque", "trq", "tq")) && !ss.hits(NOTVAL + s("mode", "curve"))) return "mcu.torque" to none
            if (ss.hits(s("throttle", "accelerator", "accel", "aps", "pedal")) && "brake" !in ss &&
                !ss.hits(NOTVAL + s("voltage", "volt", "limp", "failure")) && !hasVolt
            ) return "mcu.throttle" to none
            if (hasTemp && "motor" in ss && !notval) return "mcu.motor_temp" to none
            if (hasTemp && ctx == "motor" && ss.hits(s("controller", "ctrl", "mcu", "inverter", "igbt", "heatsink",
                    "board", "pcb", "pwr", "mosfet", "fet", "internal")) && !notval
            ) return "mcu.ctrl_temp" to none
            if (ctx == "motor" && hasCurr && "phase" in ss && !notval) return "mcu.phase_current" to none
            if (ctx == "motor" && hasVolt && !notval &&
                !ss.hits(s("phase", "q", "d", "axis", "throttle", "sensor", "supply", "ref", "ac", "theta", "elec"))
            ) return "mcu.dc_voltage" to none
            if (ctx == "motor" && hasCurr && !notval &&
                !ss.hits(s("phase", "q", "d", "axis", "sensor", "ref", "a", "b", "c", "resultant", "ac", "array", "mtpa",
                    "fw", "ia", "ib", "ic", "id", "iq", "calib"))
            ) return "mcu.dc_current" to none
        }
        if ("brake" in ss && !ss.hits(NOTVAL + s("throttle", "plaus", "pressure", "temp")) && ctx in s("motor", "vehicle")) {
            return "mcu.brake" to none
        }
        if (ss.hits(s("gear", "direction", "dnr", "fnr", "dir")) && !ss.hits(NOTVAL) && (sig.choices.isNotEmpty() || sig.length <= 4)) {
            return "mcu.gear" to none
        }
        if ("mode" in ss && ss.hits(s("drive", "ride", "riding", "driving", "eco", "sport", "custom", "vehicle")) &&
            !ss.hits(NOTVAL) && (sig.choices.isNotEmpty() || sig.length <= 4)
        ) return "mcu.mode" to none

        // ---------------- vehicle
        if (!lowhighSplit) {
            if ("speed" in ss && !ss.hits(s("motor", "engine", "fan", "pump", "rpm")) &&
                !ss.hits(NOTVAL + s("cruise", "validity", "set")) && !isB &&
                (ss.hits(s("vehicle", "veh", "wheel", "wheelbased", "front", "rear", "road", "ground")) ||
                    u in s("km/h", "m/s", "mph") || ctx == "vehicle")
            ) return "veh.speed" to none
            if ((ss.hits(s("odo", "odometer", "mileage", "vhldist")) || "total_distance" in low || "totaldistance" in low) &&
                !isB && !ss.hits(NOTVAL + s("reset", "ack", "error"))
            ) return "veh.odometer" to none
            if ("trip" in ss && !isB && !ss.hits(NOTVAL_NO_TRIP)) return "veh.trip" to none
            if ((ss.hits(s("range", "dte")) || "distance_to_empty" in low || "dist_to_empty" in low) && !isB && !ss.hits(NOTVAL)) {
                return "veh.range" to none
            }
        }
        if ((ss.hits(s("ignition", "ign", "keyon")) || ("key" in ss && ss.hits(s("status", "state", "on", "position")))) &&
            (isB || sig.choices.isNotEmpty()) && !ss.hits(s("fault", "error", "fob", "auth"))
        ) return "veh.ignition" to none
        if (ss.hits(s("stand", "sidestand", "kickstand")) && (isB || sig.choices.isNotEmpty())) return "veh.side_stand" to none
        if (ss.hits(s("lat", "latitude")) && !isB) return "veh.lat" to none
        if (ss.hits(s("lon", "lng", "longitude")) && !isB) return "veh.lon" to none

        // ---------------- battery pack V / I (after cells & charger)
        if (ctx == "battery" || ss.hits(s("pack", "batt", "battery", "bat", "bms", "total", "cumulative", "stack"))) {
            if (hasVolt && !hasTemp && !cellish && !isB && !notval && nums.isEmpty() &&
                !ss.hits(s("charger", "chg", "obc", "input", "output", "phase", "dc", "link", "bst", "bcl",
                    "imd", "relay", "yn", "rn", "bn", "throttle", "string", "str", "em", "busbar",
                    "fuse", "precharge", "pchg", "load", "host") + MAXW + MINW + STATW)
            ) return "bms.pack_voltage" to none
            if (hasCurr && !hasTemp && !isB && !notval && nums.isEmpty() &&
                !ss.hits(s("charger", "obc", "phase", "sensor", "ch", "filter", "algo", "peripheral", "pump",
                    "fb", "imd", "string", "str", "em", "heater", "fan", "host") + MAXW + MINW + STATW)
            ) return "bms.current" to none
            if (hasCurr && ss.hits(s("chg", "charge", "charging", "chglmt")) &&
                ss.hits(s("limit", "lmt", "max", "allowed", "avail", "available", "chglmt"))
            ) return "bms.chg_limit" to none
            if (hasCurr && ss.hits(s("dsg", "discharge", "dischg", "dsglmt")) &&
                ss.hits(s("limit", "lmt", "max", "allowed", "avail", "available", "dsglmt"))
            ) return "bms.dsg_limit" to none
            if ("power" in ss && !isB && !ss.hits(NOTVAL)) return "bms.power" to none
        }
        return null
    }

    private val GENERIC_PREFIX = s(
        "bms", "m", "sig", "bat", "batt", "battery", "the", "s", "str", "string", "cell", "cells",
        "v", "voltage", "volt", "vol", "temp", "temperature", "tempr", "ntc", "th", "vcell", "cellv",
        "c", "b", "msg", "info",
    )

    private fun cellKey(st: List<String>, nums: List<Long>): List<Any> {
        val prefix = ArrayList<String>()
        for (t in st) {
            if (t.isNotEmpty() && t.all { it.isDigit() }) break
            if (t in s("cell", "cells", "vcell", "cellv")) break
            if (t !in GENERIC_PREFIX) prefix += t
        }
        return listOf<Any>(prefix.joinToString("_")) + nums
    }

    // ----------------------------------------------------------------- analyzer
    private fun makeRole(key: String, family: String, sig: DbcSignal, msgName: String, index: Int? = null, label: String? = null): Role {
        val m: Meta = when (family) {
            "bms.cell_v" -> Meta("Cell", "V", "number", "battery", "cells")
            "bms.temp" -> Meta("Temp", "degC", "number", "battery", "temps")
            "bms.balance" -> Meta("Balancing", "", "bool", "battery", "balance")
            else -> ROLE_META.getValue(family)
        }
        var unit = m.unit
        var kind = m.kind
        val su = normUnit(sig.unit)
        if (unit !in s("V", "A", "degC", "km/h", "kW", "Ah", "km")) {
            unit = if (su !in s("", "V", "A") || unit == "") su else unit
            if (family == "mcu.torque" && su.isNotEmpty()) unit = su
        }
        val sk = signalKind(sig)
        if (kind == "number" && sk == "enum" && family !in NUMERIC_FAMILIES) kind = "enum"
        if (flagish(sig) && kind == "number" && family in s("bms.charge_mos", "bms.discharge_mos", "veh.ignition", "veh.side_stand")) {
            kind = "bool"
        } else if (isBool(sig) && kind == "number") {
            kind = "bool"
        }
        return Role(key, label ?: m.label, unit, kind, m.panel, m.section, index = index, source = "$msgName.${sig.name}")
    }

    private fun cmpKeyElem(a: Any, b: Any): Int {
        val ta = if (a is String) 1 else 0
        val tb = if (b is String) 1 else 0
        if (ta != tb) return ta.compareTo(tb)
        return if (a is String) a.compareTo(b as String) else (a as Long).compareTo(b as Long)
    }

    private val KEY_CMP = Comparator<List<Any>> { a, b ->
        for (i in 0 until min(a.size, b.size)) {
            val c = cmpKeyElem(a[i], b[i])
            if (c != 0) return@Comparator c
        }
        a.size.compareTo(b.size)
    }

    fun analyze(db: DbcDatabase, folderSystem: String): Analysis {
        val defaultCtx = FOLDER_SYS[folderSystem] ?: "vehicle"
        val roles = LinkedHashMap<String, Role>()
        val msgSys = LinkedHashMap<String, String>()
        val sigRole = HashMap<Pair<String, String>, String>()
        val auto = HashMap<Pair<String, String>, String>()
        val cellFound = LinkedHashMap<List<Any>, MutableList<MS>>()
        val tempFound = LinkedHashMap<List<Any>, MutableList<MS>>()
        val balFound = LinkedHashMap<List<Any>, MutableList<MS>>()
        val order = HashMap<String, Long>()
        db.messages.sortedBy { it.frameId }.forEachIndexed { i, m -> order[m.name] = i.toLong() }

        val single = LinkedHashMap<String, MutableList<MS>>()
        for (msg in db.messages) {
            val ctx = msgSystem(msg, defaultCtx)
            msgSys[msg.name] = ctx
            for (sig in msg.signals) {
                if (sig.muxSwitch) continue
                val (fam, key) = classifySignal(sig, msg, ctx) ?: continue
                if (fam in NUMERIC_FAMILIES && (signalKind(sig) == "enum" || sig.length <= 2)) continue
                when {
                    fam.startsWith("auto.") -> auto[msg.name to sig.name] = fam.substring(5)
                    fam == "bms.cell_v" -> cellFound.getOrPut(key) { ArrayList() } += msg to sig
                    fam == "bms.temp" -> tempFound.getOrPut(key) { ArrayList() } += msg to sig
                    fam == "bms.balance" -> balFound.getOrPut(key) { ArrayList() } += msg to sig
                    else -> single.getOrPut(fam) { ArrayList() } += msg to sig
                }
            }
        }

        /** Same key in different messages -> keep apart unless the messages are mirrors. */
        fun disambiguate(found: Map<List<Any>, List<MS>>): List<Pair<List<Any>, List<MS>>> {
            var items = ArrayList<Pair<List<Any>, List<MS>>>()
            for ((key, lst) in found) {
                val msgs = lst.map { it.first.name }.toSet()
                if (msgs.size > 1 && lst.size == msgs.size) {
                    for (ms in lst) items += (listOf<Any>(order.getValue(ms.first.name)) + key) to listOf(ms)
                } else {
                    items += (listOf<Any>(-1L) + key) to lst
                }
            }
            items = if (items.all { it.first[0] == -1L }) {
                ArrayList(items.map { it.first.drop(1) to it.second })
            } else {
                ArrayList(items.map { (k, v) -> (listOf<Any>(max(k[0] as Long, 0L)) + k.drop(1)) to v })
            }
            return items.sortedWith { a, b -> KEY_CMP.compare(a.first, b.first) }
        }

        fun addMulti(found: Map<List<Any>, List<MS>>, family: String, prefix: String): List<Pair<List<Any>, String>> {
            val keys = ArrayList<Pair<List<Any>, String>>()
            disambiguate(found).forEachIndexed { i0, (key, lst) ->
                val i = i0 + 1
                val rk = "$family.$i"
                val (m0, s0) = lst[0]
                val tag = key.firstOrNull { it is String && it.isNotEmpty() } as String? ?: ""
                val numsInKey = key.filterIsInstance<Long>()
                val label = if (tag.isNotEmpty() && numsInKey.isNotEmpty()) "${tag.uppercase()}·${numsInKey.last()}" else "$prefix$i"
                val role = makeRole(rk, family, s0, m0.name, index = i, label = label)
                for ((m, sg) in lst) {
                    val (f, o) = conversion(role.unit, sg, isCell = family == "bms.cell_v")
                    role.bindings += Binding(m.name, sg.name, f, o)
                    sigRole[m.name to sg.name] = rk
                }
                roles[rk] = role
                keys += key to rk
            }
            return keys
        }

        val cellKeys = addMulti(cellFound, "bms.cell_v", "C")
        val tempKeys = addMulti(tempFound, "bms.temp", "T")
        val cells = cellKeys.map { it.second }
        val temps = tempKeys.map { it.second }

        // balancing flags -> attach to the cell with the same key, else by order
        val balance = LinkedHashMap<Int, String>()
        val cellByKey = cellKeys.toMap()
        disambiguate(balFound).forEachIndexed { i0, (key, lst) ->
            val i = i0 + 1
            val rk = "bms.balance.$i"
            val (m0, s0) = lst[0]
            val role = makeRole(rk, "bms.balance", s0, m0.name, index = i, label = "Bal $i")
            for ((m, sg) in lst) {
                role.bindings += Binding(m.name, sg.name)
                sigRole[m.name to sg.name] = rk
            }
            roles[rk] = role
            val target = cellByKey[key]
            val idx = if (target != null && target in cells) cells.indexOf(target) + 1 else i
            if (idx <= cells.size) balance[idx] = rk
        }

        for ((fam, lst0) in single) {
            val lst = if (fam in NUMERIC_FAMILIES) lst0.sortedWith(compareBy({ if (it.second.unit.isNotEmpty()) 0 else 1 }, { -it.second.length })) else lst0
            val (m0, s0) = lst[0]
            val role = makeRole(fam, fam, s0, m0.name)
            for ((m, sg) in lst) {
                val (f, o) = conversion(role.unit, sg)
                role.bindings += Binding(m.name, sg.name, f, o)
                sigRole[m.name to sg.name] = fam
            }
            roles[fam] = role
        }

        // ranges / steps from the primary binding
        for (role in roles.values) {
            val b0 = role.bindings[0]
            val sig = db.byName[b0.msg]?.signals?.firstOrNull { it.name == b0.sig } ?: continue
            val (lo, hi) = sigRange(sig)
            val a = b0.toRole(lo)
            val b = b0.toRole(hi)
            var clo = min(a, b)
            var chi = max(a, b)
            val step: Double
            if (role.kind == "bool") {
                clo = 0.0; chi = 1.0; step = 1.0
            } else {
                val f = if (b0.factor == 0.0) 1.0 else b0.factor
                val st = abs(sigStep(sig) / f)
                step = if (st == 0.0) 1.0 else st
            }
            role.minimum = clo; role.maximum = chi; role.step = step
            if (role.kind == "enum" && sig.choices.isNotEmpty()) {
                role.choices = LinkedHashMap(sig.choices)
            } else if (role.kind == "number" && sig.choices.size == 1) {
                role.choices = null
            }
            saneRange(role)
        }

        for (k in listOf("bms.max_cell_v", "bms.min_cell_v", "bms.avg_cell_v", "bms.delta_cell_v", "bms.max_cell_v_id",
            "bms.min_cell_v_id", "bms.max_temp", "bms.min_temp", "bms.avg_temp", "bms.delta_temp",
            "bms.max_temp_id", "bms.min_temp_id", "bms.power")) {
            roles[k]?.derived = true
        }

        val panels = PANEL_ORDER.filter { p -> roles.values.any { it.panel == p } }
        return Analysis(roles, msgSys, sigRole, auto, panels.toMutableList(), cells, temps, balance)
    }

    private val SANE = mapOf(
        "bms.cell_v" to (0.0 to 5.0), "bms.temp" to (-40.0 to 125.0), "bms.soc" to (0.0 to 100.0), "bms.soh" to (0.0 to 100.0),
        "mcu.throttle" to (0.0 to 100.0), "bms.current" to (-1000.0 to 1000.0), "mcu.rpm" to (-20000.0 to 20000.0),
        "veh.speed" to (0.0 to 300.0), "mcu.motor_temp" to (-40.0 to 200.0), "mcu.ctrl_temp" to (-40.0 to 150.0),
        "bms.max_temp" to (-40.0 to 125.0), "bms.min_temp" to (-40.0 to 125.0), "bms.pack_voltage" to (0.0 to 1000.0),
        "bms.remaining_cap" to (0.0 to 2000.0), "bms.full_cap" to (0.0 to 2000.0), "bms.max_cell_v" to (0.0 to 5.0),
        "bms.min_cell_v" to (0.0 to 5.0), "bms.avg_cell_v" to (0.0 to 5.0), "bms.delta_cell_v" to (0.0 to 5.0),
        "mcu.dc_voltage" to (0.0 to 1000.0), "chg.out_voltage" to (0.0 to 1000.0), "chg.set_voltage" to (0.0 to 1000.0),
        "bms.cycles" to (0.0 to 20000.0), "veh.odometer" to (0.0 to 1000000.0), "mcu.dc_current" to (-1000.0 to 1000.0),
    )

    /** Clamp absurd raw ranges (e.g. 0..65535 V) to a usable UI range. */
    private fun saneRange(role: Role) {
        val fam = role.key.split(".").take(2).joinToString(".")
        val r = SANE[fam] ?: return
        if (role.kind != "number") return
        val nlo = max(role.minimum, r.first)
        val nhi = min(role.maximum, r.second)
        if (nlo < nhi) { role.minimum = nlo; role.maximum = nhi }
    }
}

class Binding(val msg: String, val sig: String, val factor: Double = 1.0, val offset: Double = 0.0) {
    fun toRole(v: Double) = if (factor != 0.0) (v - offset) / factor else v
}

class Role(
    val key: String,
    val label: String,
    val unit: String,
    val kind: String,          // number | bool | enum
    val panel: String,
    val section: String,
    var minimum: Double = 0.0,
    var maximum: Double = 1.0,
    var step: Double = 1.0,
    var choices: Map<Long, String>? = null,
    val bindings: MutableList<Binding> = ArrayList(),
    val index: Int? = null,
    val source: String = "",
    var derived: Boolean = false,
) {
    val sourceText: String get() = bindings.joinToString(", ") { "${it.msg}.${it.sig}" }
}

class Analysis(
    val roles: LinkedHashMap<String, Role>,
    val msgSystem: Map<String, String>,
    val sigRole: Map<Pair<String, String>, String>,
    val auto: Map<Pair<String, String>, String>,
    val panels: MutableList<String>,
    val cells: List<String>,
    val temps: List<String>,
    val balance: Map<Int, String>,
)
