package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Checkbox
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.core.SeriesSnap
import com.zitto.vcumaster.core.VEHICLE_ROLES
import com.zitto.vcumaster.core.VehEntry
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.ChartSeries
import com.zitto.vcumaster.ui.components.Dot
import com.zitto.vcumaster.ui.components.EmptyNote
import com.zitto.vcumaster.ui.components.Gauge
import com.zitto.vcumaster.ui.components.Note
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.PrimaryButton
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.components.SmallButton
import com.zitto.vcumaster.ui.components.StripChart
import com.zitto.vcumaster.ui.components.TileData
import com.zitto.vcumaster.ui.components.TileGrid
import com.zitto.vcumaster.ui.components.rememberNow
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono

val PLOT_COLORS = listOf(Color(0xFF2A6FDB), Color(0xFFC9711A), Color(0xFF1F8F5F), Color(0xFF8A4FD3), Color(0xFFCC3D33), Color(0xFF2B8AA8))

@Composable
fun VehicleScreen(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val now by rememberNow(100)
    val V = st.vehicle
    var mapOpen by remember { mutableStateOf(false) }
    var sigSearch by remember { mutableStateOf("") }
    var sigBus by remember { mutableIntStateOf(0) }

    fun t(e: VehEntry?, digits: Int = 1): String =
        e?.text ?: e?.value?.let { Fmt.fix(it, digits) } ?: "–"
    fun stale(e: VehEntry?) = e?.value == null || (e.ageS ?: 99.0) > 3

    ScreenColumn {
        if (st.dbcList.isEmpty()) {
            Panel("No DBC loaded") {
                Note("Vehicle values come from CAN frames decoded with a DBC. Upload one on the OTA/DBC tab, or load the demo DBC to try it with the simulator.")
                PrimaryButton("Load demo DBC", Modifier.padding(start = 14.dp, bottom = 12.dp)) { hub.loadSampleDbc() }
            }
        }

        Panel(null) {
            Row(Modifier.fillMaxWidth().padding(8.dp), horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                Gauge(V["speed"], v.can1, Modifier.weight(1f))
                Gauge(V["motor_rpm"], v.can2, Modifier.weight(1f))
                val soc = V["soc"]
                Gauge(soc, if ((soc?.value ?: 100.0) < 20) v.err else v.ok, Modifier.weight(1f))
            }
        }

        Panel("Battery and drivetrain", actions = { SmallButton("Map signals") { mapOpen = true } }) {
            val pv = V["pack_voltage"]
            val pc = V["pack_current"]
            val pvv = pv?.value
            val pcv = pc?.value
            val pw = if (pvv != null && pcv != null) pvv * pcv / 1000 else null
            TileGrid(
                listOf(
                    TileData("Pack voltage", t(pv, 2), pv?.unit ?: "V", stale(pv)),
                    TileData("Pack current", t(pc), pc?.unit ?: "A", stale(pc)),
                    TileData("Pack power", pw?.let { Fmt.fix(it, 2) } ?: "–", "kW", stale(pc)),
                    TileData("Battery temp", t(V["batt_temp"], 0), "°C", stale(V["batt_temp"])),
                    TileData("Motor temp", t(V["motor_temp"], 0), "°C", stale(V["motor_temp"])),
                    TileData("Controller temp", t(V["ctrl_temp"], 0), "°C", stale(V["ctrl_temp"])),
                    TileData("Throttle", t(V["throttle"], 0), "%", stale(V["throttle"])),
                    TileData("Brake", t(V["brake"], 0), "", stale(V["brake"])),
                    TileData("Gear / mode", t(V["gear"], 0), "", stale(V["gear"])),
                    TileData("Odometer", t(V["odometer"], 1), V["odometer"]?.unit ?: "km", stale(V["odometer"])),
                    TileData("Fault", t(V["fault"], 0), "", stale(V["fault"])),
                ),
                columns = 3,
            )
            Spacer(Modifier.width(4.dp))
        }

        Panel("Signal plot", sub = if (st.plotSel.isEmpty()) "Select up to 6 signals in the table below" else "${st.plotSel.size} of 6 signals") {
            val series = st.plotSel.mapIndexed { i, k -> ChartSeries("", PLOT_COLORS[i % 6], st.plot[k] ?: SeriesSnap.EMPTY) }
            StripChart("Decoded values", series, prefs.winS, now, height = 190.dp, showLegend = false)
            Column(Modifier.padding(horizontal = 14.dp, vertical = 4.dp)) {
                st.plotSel.forEachIndexed { i, k ->
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Dot(PLOT_COLORS[i % 6])
                        Text("  " + k.replaceFirst(":", " "), fontSize = 12.sp, fontFamily = Mono, color = v.ink2, modifier = Modifier.weight(1f))
                        TextButton(onClick = { hub.setPlotSel(st.plotSel - k) }) { Text("Remove", fontSize = 12.sp) }
                    }
                }
            }
            Seg(
                listOf(15 to "15 s", 30 to "30 s", 60 to "60 s", 120 to "2 min"), prefs.winS, { prefs.setWindow(it) },
                Modifier.fillMaxWidth().padding(start = 12.dp, end = 12.dp, bottom = 10.dp),
            )
        }

        Panel("Decoded signals") {
            Row(Modifier.fillMaxWidth().padding(horizontal = 12.dp), verticalAlignment = Alignment.CenterVertically) {
                OutlinedTextField(sigSearch, { sigSearch = it }, singleLine = true, placeholder = { Text("Filter signals", fontSize = 12.sp) }, modifier = Modifier.weight(1f))
                Spacer(Modifier.width(8.dp))
                Seg(listOf(0 to "All", 1 to "CAN1", 2 to "CAN2"), sigBus, { sigBus = it }, Modifier.width(170.dp))
            }
            var sigs = st.signals.sortedBy { it.key }
            if (sigBus != 0) sigs = sigs.filter { it.bus == sigBus }
            if (sigSearch.isNotBlank()) sigs = sigs.filter { it.key.contains(sigSearch, ignoreCase = true) }
            if (sigs.isEmpty()) EmptyNote("No decoded signals yet. Frames appear here once a DBC matches incoming CAN IDs.")
            for (s in sigs) {
                val on = s.key in st.plotSel
                Row(
                    Modifier.fillMaxWidth().clickable {
                        if (on) hub.setPlotSel(st.plotSel - s.key)
                        else if (st.plotSel.size >= 6) hub.toast("Up to 6 signals can be plotted at once. Clear one first.", true)
                        else hub.setPlotSel(st.plotSel + s.key)
                    }.padding(end = 12.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    Checkbox(checked = on, onCheckedChange = null, modifier = Modifier.padding(horizontal = 4.dp))
                    Column(Modifier.weight(1f)) {
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            Text("CAN${s.bus} ", fontSize = 11.sp, color = if (s.bus == 1) v.can1 else v.can2, fontWeight = FontWeight.SemiBold)
                            Text(s.message, fontSize = 11.sp, color = v.ink3, maxLines = 1, overflow = TextOverflow.Ellipsis)
                        }
                        Text(s.signal, fontSize = 13.5.sp, fontWeight = FontWeight.Medium, maxLines = 1, overflow = TextOverflow.Ellipsis)
                        Text(
                            "min ${Fmt.num(s.minSeen)}  max ${Fmt.num(s.maxSeen)}  ${s.count} upd  ${Fmt.ago(s.t)}",
                            fontSize = 10.5.sp, color = v.ink3, fontFamily = Mono,
                        )
                    }
                    Text(
                        (s.label ?: Fmt.num(s.value)) + if (s.unit.isNotEmpty()) " ${s.unit}" else "",
                        fontFamily = Mono, fontSize = 15.sp, fontWeight = FontWeight.Medium, color = v.ink,
                    )
                }
                HorizontalDivider(color = v.rule2)
            }
            Spacer(Modifier.width(4.dp))
        }

        Panel("CAN trace", sub = "One row per identifier") {
            val msgs = st.messages.sortedWith(compareBy({ it.bus }, { it.id }))
            if (msgs.isEmpty()) EmptyNote("No CAN frames received yet.")
            for (m in msgs) {
                Column(Modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 5.dp)) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text("CAN${m.bus} ", fontSize = 11.5.sp, color = if (m.bus == 1) v.can1 else v.can2, fontWeight = FontWeight.SemiBold)
                        Text(Fmt.hexId(m.id, m.ext), fontFamily = Mono, fontSize = 13.sp, fontWeight = FontWeight.Medium)
                        Text("  ${m.name ?: "not in DBC"}", fontSize = 12.sp, color = if (m.name == null) v.ink3 else v.ink2, modifier = Modifier.weight(1f), maxLines = 1, overflow = TextOverflow.Ellipsis)
                        Text("%.1f Hz".format(m.rateHz), fontFamily = Mono, fontSize = 11.5.sp, color = v.ink2)
                    }
                    Row {
                        Text(m.data.joinToString(" ") { Fmt.hex2(it) }, fontFamily = Mono, fontSize = 12.sp, color = v.ink, modifier = Modifier.weight(1f))
                        Text("${m.count}  ${Fmt.ago(m.last)}", fontFamily = Mono, fontSize = 11.sp, color = v.ink3)
                    }
                }
                HorizontalDivider(color = v.rule2)
            }
            Spacer(Modifier.width(4.dp))
        }
    }

    if (mapOpen) MapDialog(st, hub) { mapOpen = false }
}

@Composable
private fun MapDialog(st: HubState, hub: Hub, onDismiss: () -> Unit) {
    val v = LocalVcu.current
    AlertDialog(
        onDismissRequest = onDismiss,
        confirmButton = { TextButton(onClick = onDismiss) { Text("Done") } },
        title = { Text("Signal mapping") },
        text = {
            Column {
                Text("Pick which DBC signal feeds each vehicle value.", fontSize = 12.sp, color = v.ink3)
                LazyColumn(Modifier.heightIn(max = 460.dp)) {
                    items(VEHICLE_ROLES.entries.toList(), key = { it.key }) { (role, meta) ->
                        var open by remember { mutableStateOf(false) }
                        val cur = st.vehicleMap[role]
                        Box {
                            Column(Modifier.fillMaxWidth().clickable { open = true }.padding(vertical = 6.dp)) {
                                Text(meta.label, fontSize = 13.sp, color = v.ink2)
                                Text(cur ?: "Not mapped", fontFamily = Mono, fontSize = 12.5.sp, color = if (cur == null) v.ink3 else v.focus)
                            }
                            DropdownMenu(expanded = open, onDismissRequest = { open = false }) {
                                DropdownMenuItem(text = { Text("Not mapped") }, onClick = { hub.setVehicleMap(role, null); open = false })
                                for (s in st.allSignals) {
                                    DropdownMenuItem(
                                        text = { Text(s, fontFamily = Mono, fontSize = 12.5.sp, color = if (s == cur) v.focus else v.ink) },
                                        onClick = { hub.setVehicleMap(role, s); open = false },
                                    )
                                }
                            }
                        }
                        HorizontalDivider(color = v.rule2)
                    }
                }
            }
        },
    )
}
