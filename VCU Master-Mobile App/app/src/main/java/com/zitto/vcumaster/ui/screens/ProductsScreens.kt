package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.FlagRef
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.core.Role
import com.zitto.vcumaster.core.RolesMeta
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.EmptyNote
import com.zitto.vcumaster.ui.components.Note
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.PrimaryButton
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono
import java.util.Locale
import kotlin.math.max
import kotlin.math.min
import kotlin.math.roundToInt

/*
 * Battery (BMS) and Motor (MCU) windows (V0.0073, port of static/products.js): laid out from the
 * loaded DBCs by the same analyzer as CAN_DBC_Simulator; read-only, values come from decoded CAN.
 */

/** Role values + ages and the latest decoded value of every "Msg.Sig" (for the fault flags). */
private class RoleView(st: HubState) {
    val meta: RolesMeta = st.roles
    val values = st.roleValues
    val age = st.roleAge
    val sig: Map<String, Double?> = HashMap<String, Pair<Double, Double?>>().also { o ->
        for (s in st.signals) {
            val k = "${s.message}.${s.signal}"
            val prev = o[k]
            if (prev == null || s.t > prev.first) o[k] = s.t to s.value
        }
    }.mapValues { it.value.second }

    fun role(k: String): Role? = meta.roles[k]
    fun v(k: String): Double? = values[k]
    fun stale(k: String): Boolean = (age[k] ?: 99.0) > 5
    fun has(panel: String) = panel in meta.panels
}

private fun unitText(u: String) = if (u == "degC") "°C" else u
private fun fmt(v: Double?, d: Int) = if (v == null || v.isNaN()) "–" else String.format(Locale.US, "%.${d}f", v)

private fun roleText(r: Role, v: Double?): String {
    if (v == null) return "–"
    if (r.kind == "bool") return if (v != 0.0) "ON" else "OFF"
    if (r.kind == "enum") r.choices?.let { c -> return c[v.roundToInt().toLong()] ?: fmt(v, 0) }
    return fmt(v, if (r.step >= 1 || r.section == "count") 0 else if (r.unit == "V") 3 else 1)
}

@Composable
private fun NoProduct(text: String, onGoto: () -> Unit) {
    Panel(null) {
        Text(text, fontSize = 14.sp, lineHeight = 19.sp, modifier = Modifier.padding(14.dp))
        PrimaryButton("Open OTA and DBC", Modifier.padding(start = 14.dp, bottom = 14.dp)) { onGoto() }
    }
}

/** Value tile with the DBC signal(s) it comes from. */
@Composable
private fun RoleTile(rv: RoleView, key: String, modifier: Modifier) {
    val v = LocalVcu.current
    val r = rv.role(key) ?: return
    val value = rv.v(key)
    val stale = rv.stale(key)
    val on = r.kind == "bool" && value != null && value != 0.0
    Column(
        modifier.padding(3.dp)
            .background(if (on) v.ok.copy(alpha = 0.12f) else if (v.dark) v.rule2 else Color(0xFFF6F8F9), RoundedCornerShape(8.dp))
            .padding(horizontal = 10.dp, vertical = 7.dp),
    ) {
        Text(r.label, fontSize = 11.5.sp, color = v.ink3, maxLines = 1, overflow = TextOverflow.Ellipsis)
        Row(verticalAlignment = Alignment.Bottom) {
            Text(
                roleText(r, value), fontFamily = Mono, fontSize = 18.sp, fontWeight = FontWeight.Medium,
                color = if (stale) v.ink3 else if (on) v.ok else v.ink, maxLines = 1, overflow = TextOverflow.Ellipsis,
                modifier = Modifier.weight(1f, fill = false),
            )
            if (r.kind == "number" && r.unit.isNotEmpty()) Text(" " + unitText(r.unit), fontSize = 11.sp, color = v.ink3, maxLines = 1)
        }
        Text(r.sourceText, fontSize = 9.5.sp, color = v.ink3, fontFamily = Mono, maxLines = 1, overflow = TextOverflow.Ellipsis)
    }
}

@Composable
private fun RoleTiles(rv: RoleView, keys: List<String>, columns: Int = 2) {
    val present = keys.filter { rv.role(it) != null }
    Column(Modifier.fillMaxWidth().padding(horizontal = 8.dp, vertical = 4.dp)) {
        for (row in present.chunked(columns)) {
            Row(Modifier.fillMaxWidth()) {
                for (k in row) RoleTile(rv, k, Modifier.weight(1f))
                repeat(columns - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
    }
}

/** Full-circle meter like the desktop ring (SOC, motor speed, torque, vehicle speed). */
@Composable
private fun RoleRing(rv: RoleView, key: String, label: String?, color: Color, modifier: Modifier) {
    val v = LocalVcu.current
    val r = rv.role(key) ?: return
    val tm = rememberTextMeasurer()
    val value = rv.v(key)
    val stale = rv.stale(key)
    val lo = r.minimum
    val hi = r.maximum
    val f = if (value == null) 0f else ((value - lo) / (if (hi - lo == 0.0) 1.0 else hi - lo)).toFloat().coerceIn(0f, 1f)
    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Canvas(Modifier.fillMaxWidth().aspectRatio(1f).padding(6.dp)) {
            val stroke = size.width * 0.085f
            val d = size.width - stroke
            val tl = Offset(stroke / 2, stroke / 2)
            drawArc(v.rule2, -90f, 360f, false, tl, Size(d, d), style = Stroke(stroke))
            if (f > 0.001f) drawArc(if (stale) v.ink3 else color, -90f, 360f * f, false, tl, Size(d, d), style = Stroke(stroke, cap = StrokeCap.Round))
            val txt = fmt(value, if (r.step >= 1) 0 else 1)
            val lay = tm.measure(txt, TextStyle(fontFamily = Mono, fontSize = (size.width / 5.2f).toSp(), fontWeight = FontWeight.Medium, color = if (stale) v.ink3 else v.ink))
            drawText(lay, topLeft = Offset(size.width / 2 - lay.size.width / 2, size.height / 2 - lay.size.height * 0.62f))
            val ul = tm.measure(unitText(r.unit), TextStyle(fontSize = 11.sp, color = v.ink3))
            drawText(ul, topLeft = Offset(size.width / 2 - ul.size.width / 2, size.height / 2 + lay.size.height * 0.36f))
        }
        Text(label ?: r.label, fontSize = 12.sp, color = v.ink2, maxLines = 1, overflow = TextOverflow.Ellipsis, textAlign = TextAlign.Center)
        Text(r.sourceText, fontSize = 9.5.sp, color = v.ink3, fontFamily = Mono, maxLines = 1, overflow = TextOverflow.Ellipsis)
    }
}

// ------------------------------------------------------------------------- faults and status
private val FAULT_RE = Regex("fault|err|fail|alarm|warn|protect|over|under|short|high|low", RegexOption.IGNORE_CASE)

@Composable
private fun FlagsPanel(rv: RoleView, panel: String, prefs: UiPrefs) {
    val v = LocalVcu.current
    val list = rv.meta.flags[panel].orEmpty()
    if (list.isEmpty()) return
    fun value(f: FlagRef) = rv.sig["${f.msg}.${f.sig}"]
    val shown = if (prefs.flagsActive) list.filter { (value(it) ?: 0.0) != 0.0 } else list
    Panel("Faults and status", sub = "${list.size} from the DBC · filled dot = active, red = active fault", actions = {
        Seg(listOf(false to "All", true to "Active"), prefs.flagsActive, { prefs.showActiveFlags(it) })
    }) {
        if (shown.isEmpty()) EmptyNote("No flag is active.")
        for (row in shown.chunked(2)) {
            Row(Modifier.fillMaxWidth().padding(horizontal = 10.dp, vertical = 2.dp)) {
                for (f in row) {
                    val x = value(f)
                    val on = x != null && x != 0.0
                    val fault = FAULT_RE.containsMatchIn(f.sig)
                    val c = if (on) (if (fault) v.err else v.ok) else v.ink3
                    val txt = if (x == null) "–" else if (f.kind == "enum") f.choices[x.roundToInt().toLong()] ?: fmt(x, 0) else fmt(x, 0)
                    Row(Modifier.weight(1f).padding(vertical = 3.dp, horizontal = 2.dp), verticalAlignment = Alignment.CenterVertically) {
                        Box(
                            Modifier.size(9.dp).then(if (on) Modifier.background(c, CircleShape) else Modifier.border(1.5.dp, c, CircleShape)),
                        )
                        Spacer(Modifier.width(6.dp))
                        Text(f.sig, fontSize = 11.5.sp, color = if (on) v.ink else v.ink2, maxLines = 1, overflow = TextOverflow.Ellipsis, modifier = Modifier.weight(1f))
                        Text(txt, fontSize = 11.sp, fontFamily = Mono, color = c, maxLines = 1, modifier = Modifier.padding(start = 4.dp))
                    }
                }
                if (row.size == 1) Spacer(Modifier.weight(1f))
            }
        }
        Spacer(Modifier.height(8.dp))
    }
}

// ------------------------------------------------------------------------- battery
private val LAYOUTS = listOf("auto", "4", "7", "8", "10", "12", "13", "14", "15", "16", "17", "20", "24", "28", "32", "48", "64", "96", "128", "144", "160", "192")
private val KPI = listOf("bms.pack_voltage", "bms.current", "bms.power", "bms.soh", "bms.remaining_cap", "bms.full_cap", "bms.cycles",
    "bms.chg_limit", "bms.dsg_limit", "bms.cell_count", "bms.temp_count", "bms.charge_mos", "bms.discharge_mos")
private val STATS = listOf("bms.max_cell_v", "bms.min_cell_v", "bms.avg_cell_v", "bms.delta_cell_v", "bms.max_cell_v_id", "bms.min_cell_v_id",
    "bms.max_temp", "bms.min_temp", "bms.avg_temp", "bms.delta_temp")

private class Geometry(val cells: List<String>, val temps: List<String>, val repC: Int, val repT: Int, val autoN: Int, val slots: Int, val tslots: Int)

private fun geometry(rv: RoleView, layout: String): Geometry {
    fun countOf(key: String, maxN: Int): Int {
        val x = rv.v(key) ?: return 0
        return if (x in 1.0..512.0 && !rv.stale(key)) min(x.roundToInt(), max(maxN, 1) * 4) else 0
    }
    val cells = rv.meta.cells
    val temps = rv.meta.temps
    val repC = countOf("bms.cell_count", cells.size)
    val repT = countOf("bms.temp_count", temps.size)
    var seen = 0
    cells.forEachIndexed { i, k -> if ((rv.v(k) ?: 0.0) > 0) seen = i + 1 }          // highest cell with data
    var seenT = 0
    temps.forEachIndexed { i, k -> if (rv.v(k) != null && !rv.stale(k)) seenT = i + 1 }
    val autoN = if (repC > 0) repC else if (seen > 0) seen else cells.size
    val slots = if (layout == "auto") autoN else layout.toIntOrNull() ?: autoN
    val tslots = if (repT > 0) repT else if (temps.size > 8 && seenT > 0) seenT else temps.size
    return Geometry(cells, temps, repC, repT, autoN, slots, tslots)
}

@Composable
fun BatteryScreen(st: HubState, prefs: UiPrefs, onGoto: (String) -> Unit) {
    val v = LocalVcu.current
    val rv = RoleView(st)
    ScreenColumn {
        if (!rv.has("battery")) {
            NoProduct(
                "No battery signals yet. VCU Master loads a matching DBC from its library automatically when BMS " +
                    "frames arrive, or pick one in OTA and DBC.",
            ) { onGoto("updates") }
            return@ScreenColumn
        }
        val g = geometry(rv, prefs.bmsLayout)

        Panel("Pack", sub = "DBC ${rv.meta.dbcs.joinToString(", ")}") {
            Row(Modifier.fillMaxWidth().padding(horizontal = 8.dp), verticalAlignment = Alignment.CenterVertically) {
                if (rv.role("bms.soc") != null) RoleRing(rv, "bms.soc", "State of charge", v.ok, Modifier.width(150.dp))
            }
            RoleTiles(rv, KPI)
            Spacer(Modifier.height(6.dp))
        }

        CellPanel(rv, g, prefs)

        if (g.tslots > 0) {
            Panel("Temperatures", sub = "${g.tslots}${if (g.repT > 0) " reported by the BMS" else ""} of ${g.temps.size} in the DBC") {
                CellGrid(g.temps.take(g.tslots).size) { i, mod ->
                    val k = g.temps[i]
                    val x = rv.v(k)
                    val stale = rv.stale(k)
                    CellBox(
                        label = rv.role(k)?.label ?: "T${i + 1}", value = fmt(x, 1), unit = "°C",
                        frac = if (x == null) 0f else ((x + 20) / 110).toFloat().coerceIn(0.02f, 1f),
                        color = if ((x ?: 0.0) > 50) v.err else v.csa, stale = stale, mark = null, balancing = null, modifier = mod,
                    )
                }
                Spacer(Modifier.height(8.dp))
            }
        }
        if (STATS.any { rv.role(it) != null }) {
            Panel("Cell statistics", sub = "as reported by the BMS") {
                RoleTiles(rv, STATS)
                Spacer(Modifier.height(6.dp))
            }
        }
        FlagsPanel(rv, "battery", prefs)
    }
}

@Composable
private fun CellPanel(rv: RoleView, g: Geometry, prefs: UiPrefs) {
    val v = LocalVcu.current
    var menu by remember { mutableStateOf(false) }
    // live values of the fitted cells, for max / min / spread
    val vs = ArrayList<Double>()
    for (i in 0 until min(g.slots, g.cells.size)) {
        if (g.repC > 0 && i >= g.repC) continue
        val x = rv.v(g.cells[i])
        if (x != null && x > 0 && !rv.stale(g.cells[i])) vs += x
    }
    val vmax = vs.maxOrNull()
    val vmin = vs.minOrNull()
    val info = if (vs.isNotEmpty()) {
        "${vs.size} live of ${g.slots} shown · max ${(vmax!! * 1000).roundToInt()} mV · min ${(vmin!! * 1000).roundToInt()} mV · " +
            "spread ${((vmax - vmin) * 1000).roundToInt()} mV · average ${(vs.average() * 1000).roundToInt()} mV · faded = no update for 5 s"
    } else "${g.slots} slot(s) · waiting for cell frames"

    fun mark(i: Int, x: Double?, stale: Boolean): String? =
        if (!stale && x != null && vs.size > 1 && vmax != vmin) (if (x == vmax) "MAX" else if (x == vmin) "MIN" else null) else null
    fun bal(i: Int): Boolean? = rv.meta.balance[i + 1]?.let { (rv.v(it) ?: 0.0) != 0.0 }

    Panel("Cell voltages", actions = {
        Box {
            TextButton(onClick = { menu = true }) {
                Text(if (prefs.bmsLayout == "auto") "Auto (${g.autoN})" else "${prefs.bmsLayout} cells", fontSize = 12.sp)
                Icon(Icons.Filled.ExpandMore, null)
            }
            DropdownMenu(expanded = menu, onDismissRequest = { menu = false }) {
                for (l in LAYOUTS) {
                    val label = if (l == "auto") "Auto (${g.autoN}${if (g.repC > 0) ", reported by BMS" else ""})" else "$l cells"
                    DropdownMenuItem(text = { Text(label + if (l == prefs.bmsLayout) "  ✓" else "") }, onClick = { prefs.setLayout(l); menu = false })
                }
            }
        }
        Seg(listOf("grid" to "Grid", "table" to "Table"), prefs.bmsView, { prefs.setView(it) })
    }) {
        Note(info)
        if (prefs.bmsView == "table") {
            Row(Modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 4.dp)) {
                for ((h, w) in listOf("Cell" to 0.8f, "mV" to 0.8f, "Mark" to 0.7f, "Bal." to 0.7f, "Signal" to 2.2f)) {
                    Text(h, fontSize = 11.sp, color = v.ink3, fontWeight = FontWeight.SemiBold, modifier = Modifier.weight(w))
                }
            }
            for (i in 0 until g.slots) {
                val k = g.cells.getOrNull(i) ?: continue
                if (g.repC > 0 && i >= g.repC) continue
                val r = rv.role(k) ?: continue
                val x = rv.v(k)
                val stale = rv.stale(k) || !(x != null && x > 0)
                val b = bal(i)
                HorizontalDivider(color = v.rule2)
                Row(Modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 5.dp), verticalAlignment = Alignment.CenterVertically) {
                    val c = if (stale) v.ink3 else v.ink
                    Text(r.label, fontSize = 12.sp, color = c, modifier = Modifier.weight(0.8f))
                    Text(x?.let { "${(it * 1000).roundToInt()}" } ?: "–", fontSize = 12.5.sp, fontFamily = Mono, color = c, modifier = Modifier.weight(0.8f))
                    Text(mark(i, x, stale) ?: "", fontSize = 11.sp, color = if (mark(i, x, stale) == "MAX") v.err else v.focus, modifier = Modifier.weight(0.7f))
                    Text(if (b == null) "" else if (b) "active" else "off", fontSize = 11.sp, color = if (b == true) v.warn else v.ink3, modifier = Modifier.weight(0.7f))
                    Text(r.sourceText, fontSize = 10.sp, fontFamily = Mono, color = v.ink3, maxLines = 1, overflow = TextOverflow.Ellipsis, modifier = Modifier.weight(2.2f))
                }
            }
        } else {
            CellGrid(g.slots) { i, mod ->
                val k = g.cells.getOrNull(i)
                val state = if (k == null) "nodbc" else if (g.repC > 0 && i >= g.repC) "unfitted" else null
                if (state != null) {
                    CellBox(
                        label = k?.let { rv.role(it)?.label } ?: "C${i + 1}", value = "N/A",
                        unit = if (state == "nodbc") "not in DBC" else "not fitted",
                        frac = 0f, color = v.ink3, stale = true, mark = null, balancing = null, modifier = mod, off = true,
                    )
                } else {
                    val x = rv.v(k!!)
                    val stale = rv.stale(k) || !(x != null && x > 0)
                    val m = mark(i, x, stale)
                    CellBox(
                        label = rv.role(k)?.label ?: "C${i + 1}", value = x?.let { "${(it * 1000).roundToInt()}" } ?: "–", unit = "mV",
                        frac = if (x != null && x > 0) ((x - 2.5) / (4.3 - 2.5)).toFloat().coerceIn(0.02f, 1f) else 0f,
                        color = if (m == "MAX") v.err else if (m == "MIN") v.focus else v.ok,
                        stale = stale, mark = m, balancing = bal(i), modifier = mod,
                    )
                }
            }
        }
        Spacer(Modifier.height(8.dp))
    }
}

/** 4-column grid of cells / temperature sensors. */
@Composable
private fun CellGrid(n: Int, item: @Composable (Int, Modifier) -> Unit) {
    Column(Modifier.fillMaxWidth().padding(horizontal = 8.dp)) {
        for (row in (0 until n).chunked(4)) {
            Row(Modifier.fillMaxWidth()) {
                for (i in row) item(i, Modifier.weight(1f))
                repeat(4 - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
    }
}

@Composable
private fun CellBox(
    label: String, value: String, unit: String, frac: Float, color: Color, stale: Boolean,
    mark: String?, balancing: Boolean?, modifier: Modifier, off: Boolean = false,
) {
    val v = LocalVcu.current
    val border = when (mark) { "MAX" -> v.err; "MIN" -> v.focus; else -> if (balancing == true) v.warn else v.rule2 }
    Column(
        modifier.padding(2.5.dp).border(1.dp, border, RoundedCornerShape(8.dp))
            .background(if (off) Color.Transparent else if (v.dark) v.rule2.copy(alpha = 0.5f) else Color(0xFFF8FAFB), RoundedCornerShape(8.dp))
            .padding(horizontal = 6.dp, vertical = 5.dp),
    ) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(label, fontSize = 10.5.sp, color = v.ink3, maxLines = 1, overflow = TextOverflow.Ellipsis, modifier = Modifier.weight(1f))
            if (mark != null) Text(mark, fontSize = 8.5.sp, color = border, fontWeight = FontWeight.Bold)
        }
        Text(
            value, fontFamily = Mono, fontSize = 14.sp, fontWeight = FontWeight.Medium, maxLines = 1,
            color = if (off || stale) v.ink3 else v.ink,
        )
        Box(Modifier.fillMaxWidth().height(4.dp).background(v.rule2, RoundedCornerShape(2.dp))) {
            if (frac > 0f) Box(Modifier.fillMaxWidth(frac).height(4.dp).background(if (stale) v.ink3 else color, RoundedCornerShape(2.dp)))
        }
        Text(
            if (balancing == true) "balancing" else unit, fontSize = 9.sp, maxLines = 1, overflow = TextOverflow.Ellipsis,
            color = if (balancing == true) v.warn else v.ink3,
        )
    }
}

// ------------------------------------------------------------------------- motor
@Composable
fun MotorScreen(st: HubState, prefs: UiPrefs, onGoto: (String) -> Unit) {
    val v = LocalVcu.current
    val rv = RoleView(st)
    ScreenColumn {
        if (!rv.has("motor")) {
            NoProduct(
                "No motor-controller signals yet. A matching MCU DBC is loaded automatically when its frames arrive, " +
                    "or pick one in OTA and DBC.",
            ) { onGoto("updates") }
            return@ScreenColumn
        }
        val gauges = listOf("mcu.rpm" to v.can1, "mcu.torque" to v.can2, "veh.speed" to v.imu).filter { rv.role(it.first) != null }
        Panel("Motor", sub = "DBC ${rv.meta.dbcs.joinToString(", ")}") {
            Row(Modifier.fillMaxWidth().padding(horizontal = 8.dp), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                for ((k, c) in gauges) RoleRing(rv, k, null, c, Modifier.weight(1f))
                repeat(3 - gauges.size) { Spacer(Modifier.weight(1f)) }
            }
            Spacer(Modifier.height(6.dp))
        }
        Panel("Drive, power and thermal") {
            RoleTiles(
                rv,
                listOf("mcu.throttle", "mcu.brake", "mcu.gear", "mcu.mode", "mcu.dc_voltage", "mcu.dc_current", "mcu.phase_current",
                    "mcu.motor_temp", "mcu.ctrl_temp", "veh.odometer", "veh.trip"),
            )
            if (listOf("mcu.throttle", "mcu.dc_voltage", "mcu.motor_temp", "veh.odometer").none { rv.role(it) != null }) {
                EmptyNote("The loaded DBC has no drive, power or thermal signals.")
            }
            Spacer(Modifier.height(6.dp))
        }
        FlagsPanel(rv, "motor", prefs)
    }
}
