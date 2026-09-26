package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.imePadding
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardActions
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.Send
import androidx.compose.material.icons.filled.ExpandLess
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material.icons.filled.History
import androidx.compose.material3.AssistChip
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Switch
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.ImeAction
import androidx.compose.ui.text.input.KeyboardCapitalization
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.link.SIM_ADDRESS
import com.zitto.vcumaster.link.ScanDev
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.Badge
import com.zitto.vcumaster.ui.components.BadgeKind
import com.zitto.vcumaster.ui.components.ConsoleBox
import com.zitto.vcumaster.ui.components.EmptyNote
import com.zitto.vcumaster.ui.components.KvList
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.PrimaryButton
import com.zitto.vcumaster.ui.components.RssiBars
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.components.SmallButton
import com.zitto.vcumaster.ui.components.rememberNow
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono

@Composable
fun ScreenColumn(content: @Composable () -> Unit) {
    Column(
        Modifier.fillMaxSize().verticalScroll(rememberScrollState()).imePadding().padding(horizontal = 12.dp, vertical = 10.dp),
        verticalArrangement = Arrangement.spacedBy(12.dp),
    ) {
        content()
        Spacer(Modifier.height(8.dp))
    }
}

@Composable
fun ConnectScreen(st: HubState, hub: Hub, prefs: UiPrefs, gate: ((() -> Unit) -> Unit)) {
    val v = LocalVcu.current
    val scan by hub.scan.collectAsStateWithLifecycle()
    var connectingAddr by remember { mutableStateOf<String?>(null) }
    val now by rememberNow(1000)
    val L = st.link

    fun doConnect(d: ScanDev) {
        val go = {
            connectingAddr = d.address
            hub.act("Connected to ${d.name}") {
                try { this.connect(d.address, d.name, prefs.autoRe) } finally { connectingAddr = null }
            }
        }
        if (d.address == SIM_ADDRESS) go() else gate(go)
    }

    ScreenColumn {
        Panel("Find a bridge", sub = "Nordic UART bridges are listed first") {
            Row(Modifier.fillMaxWidth().padding(horizontal = 12.dp), verticalAlignment = Alignment.CenterVertically) {
                Text("Scan for", fontSize = 13.sp, color = v.ink2)
                Spacer(Modifier.width(8.dp))
                Seg(listOf(3 to "3 s", 5 to "5 s", 10 to "10 s"), prefs.scanTime, { prefs.setScanSeconds(it) }, Modifier.weight(1f))
                Spacer(Modifier.width(8.dp))
                PrimaryButton(if (scan.scanning) "Scanning…" else "Scan", enabled = !scan.scanning) {
                    gate { hub.startScan(prefs.scanTime, prefs.nameFilter, prefs.onlyBridge) }
                }
            }
            OutlinedTextField(
                value = prefs.nameFilter, onValueChange = { prefs.setName(it) },
                label = { Text("Name contains…") }, singleLine = true,
                modifier = Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 6.dp),
            )
            SwitchRow("Only Zitto bridges", prefs.onlyBridge) { prefs.setOnly(it) }
            SwitchRow("Reconnect automatically", prefs.autoRe) { prefs.setAuto(it) }
            val devs = scan.devices
            scan.note?.let { Text(it, fontSize = 12.5.sp, lineHeight = 17.sp, color = v.warn, modifier = Modifier.padding(horizontal = 14.dp, vertical = 6.dp)) }
            if (devs == null) {
                EmptyNote("Run a scan to list nearby BLE devices. The simulator is always available for testing without hardware.")
                DeviceRow(com.zitto.vcumaster.link.BleScanner.simDevice(), connectingAddr) { doConnect(it) }
            } else if (devs.isEmpty()) {
                EmptyNote("No devices found. Check the ESP32 is powered and advertising as Zitto_MB_V1_Bridge, then scan again.")
            } else {
                for (d in devs) DeviceRow(d, connectingAddr) { doConnect(it) }
            }
            val last = st.lastDevice
            if (last != null && !L.connected && devs == null) {
                Row(Modifier.padding(horizontal = 12.dp, vertical = 6.dp), verticalAlignment = Alignment.CenterVertically) {
                    Text("Last used: ${last.second ?: ""} ${last.first}", fontSize = 12.sp, color = v.ink3, modifier = Modifier.weight(1f))
                    SmallButton("Reconnect") {
                        doConnect(ScanDev(last.first, last.second ?: last.first, null, null, emptyList(), emptyMap(), true))
                    }
                }
            }
            Spacer(Modifier.height(8.dp))
        }

        Panel("Link", actions = {
            SmallButton("Disconnect", enabled = L.connected || L.connecting) { hub.act("Disconnected") { disconnect() } }
        }) {
            val on = L.connected
            val kv = mutableListOf(
                "State" to (if (on) "Connected" else if (L.connecting) "Connecting…" else "Disconnected"),
                "Device" to (L.name ?: "–"),
                "Address" to (L.address ?: "–"),
                "Transport" to when (L.kind) { "sim" -> "Simulator"; "ble" -> "Bluetooth LE"; else -> "–" },
                "ATT MTU" to (L.mtu?.let { "$it (max write ${it - 3} B)" } ?: "–"),
                "Connected for" to (if (on && L.since != null) Fmt.dur((now - L.since) * 1000) else "–"),
                "Lines received" to Fmt.thousands(L.rxLines),
                "Commands sent" to "${L.txCmds}${if (L.txErrors > 0) ", ${L.txErrors} failed" else ""}",
                "Last data" to Fmt.ago(L.lastRx),
                "Reconnects" to "${L.reconnects}",
            )
            L.error?.let { kv += "Last error" to it }
            KvList(kv)
            var gattOpen by remember { mutableStateOf(false) }
            Row(
                Modifier.fillMaxWidth().clickable { gattOpen = !gattOpen }.padding(horizontal = 14.dp, vertical = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Text("GATT services", fontSize = 13.sp, color = v.ink2, modifier = Modifier.weight(1f))
                Icon(if (gattOpen) Icons.Filled.ExpandLess else Icons.Filled.ExpandMore, null, tint = v.ink3)
            }
            if (gattOpen) {
                if (st.services.isEmpty()) EmptyNote("Connect to list services.")
                for (s in st.services) {
                    Column(Modifier.padding(horizontal = 14.dp, vertical = 4.dp)) {
                        Text("${s.uuid}  ${s.description}", fontFamily = Mono, fontSize = 11.5.sp, color = v.ink)
                        for (c in s.chars) {
                            Text(
                                "   ${c.uuid} [${c.properties.joinToString(", ")}] ${c.description}",
                                fontFamily = Mono, fontSize = 11.sp, color = v.ink3,
                            )
                        }
                    }
                }
                Spacer(Modifier.height(8.dp))
            }
        }

        IntegrityPanel(st)

        BridgeConsole(st, hub, prefs)
    }
}

/** V0.0073: frames lost between the S32K and this phone, from the S32K frame sequence numbers. */
@Composable
private fun IntegrityPanel(st: HubState) {
    val I = st.integrity
    val badge = when {
        I.s32Frames == 0L -> "NO DATA" to BadgeKind.NONE
        I.bad -> "LOSS DETECTED" to BadgeKind.ERR
        else -> "NO LOSS" to BadgeKind.OK
    }
    Panel(
        "Data integrity", sub = "S32K → UART → ESP32 → BLE → phone, counted from the S32K frame sequence numbers since connect / clear",
        actions = { Badge(badge.first, badge.second) },
    ) {
        fun can(c: com.zitto.vcumaster.core.CanInteg) = "${c.received} / ${c.s32Counted}" + if (c.missing > 0) " (missing ${c.missing})" else ""
        val rows = mutableListOf(
            "S32K frames received" to Fmt.thousands(I.s32Frames),
            "S32K frames lost (sequence gaps)" to "${I.s32Lost} (${I.lossPct} %)",
            "CAN1 received / S32K counted" to can(I.can1),
            "CAN2 received / S32K counted" to can(I.can2),
        )
        for ((k, x) in I.bridge) rows += "ESP32 ${k.replace('_', ' ')}" to x
        KvList(rows, columns = 2)
        Spacer(Modifier.height(6.dp))
    }
}

@Composable
private fun SwitchRow(label: String, checked: Boolean, onChange: (Boolean) -> Unit) {
    Row(
        Modifier.fillMaxWidth().clickable { onChange(!checked) }.padding(horizontal = 14.dp, vertical = 2.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Text(label, fontSize = 14.sp, modifier = Modifier.weight(1f))
        Switch(checked = checked, onCheckedChange = onChange)
    }
}

@Composable
private fun DeviceRow(d: ScanDev, connecting: String?, onConnect: (ScanDev) -> Unit) {
    val v = LocalVcu.current
    Row(
        Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 4.dp)
            .border(1.dp, if (d.bridge) v.imu.copy(alpha = 0.55f) else v.rule2, RoundedCornerShape(10.dp))
            .background(if (d.bridge) v.imu.copy(alpha = 0.06f) else v.panel, RoundedCornerShape(10.dp))
            .padding(horizontal = 12.dp, vertical = 8.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Column(Modifier.weight(1f)) {
            Text(d.name, fontWeight = FontWeight.Medium, fontSize = 14.sp, maxLines = 1, overflow = TextOverflow.Ellipsis)
            Text(d.address, fontFamily = Mono, fontSize = 11.5.sp, color = v.ink3)
            RssiBars(d.rssi)
        }
        if (connecting == d.address) {
            CircularProgressIndicator(Modifier.size(24.dp), strokeWidth = 2.dp)
        } else if (d.bridge) {
            PrimaryButton("Connect") { onConnect(d) }
        } else {
            SmallButton("Connect") { onConnect(d) }
        }
    }
}

@Composable
private fun BridgeConsole(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    var input by remember { mutableStateOf("") }
    var hist by remember { mutableStateOf(false) }

    fun submit() {
        val c = input.trim()
        if (c.isEmpty()) return
        prefs.pushHist(c)
        hub.act { send(c) }
        input = ""
    }

    Panel("Bridge console", sub = "Replies, commands and errors") {
        Row(Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()).padding(horizontal = 10.dp), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
            for ((cmd, label) in listOf("PING" to "Ping", "INFO" to "Info", "STATS" to "Stats", "GPIO" to "GPIO map", "RAW:03" to "Status request")) {
                AssistChip(onClick = { hub.sendCmd(cmd) }, label = { Text(label, fontSize = 12.sp) })
            }
        }
        ConsoleBox(st.console, 280.dp, "Bridge replies appear here.")
        Row(Modifier.fillMaxWidth().padding(start = 10.dp, end = 4.dp, bottom = 10.dp, top = 4.dp), verticalAlignment = Alignment.CenterVertically) {
            OutlinedTextField(
                value = input, onValueChange = { input = it },
                placeholder = { Text("PING, ESP:13:1, S32:3:1:1, RAW:010201", fontSize = 12.sp, fontFamily = Mono) },
                singleLine = true,
                textStyle = androidx.compose.ui.text.TextStyle(fontFamily = Mono, fontSize = 13.sp, color = v.ink),
                keyboardOptions = KeyboardOptions(capitalization = KeyboardCapitalization.Characters, autoCorrectEnabled = false, imeAction = ImeAction.Send),
                keyboardActions = KeyboardActions(onSend = { submit() }),
                modifier = Modifier.weight(1f),
            )
            Box {
                IconButton(onClick = { hist = true }, enabled = prefs.cmdHist.isNotEmpty()) { Icon(Icons.Filled.History, "History") }
                DropdownMenu(expanded = hist, onDismissRequest = { hist = false }) {
                    for (h in prefs.cmdHist.reversed()) {
                        DropdownMenuItem(text = { Text(h, fontFamily = Mono, fontSize = 13.sp) }, onClick = { input = h; hist = false })
                    }
                }
            }
            IconButton(onClick = { submit() }) { Icon(Icons.AutoMirrored.Filled.Send, "Send", tint = v.focus) }
        }
    }
}
