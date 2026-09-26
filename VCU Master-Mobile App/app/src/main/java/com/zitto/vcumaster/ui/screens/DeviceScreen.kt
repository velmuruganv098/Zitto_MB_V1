package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.IntrinsicSize
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.verticalScroll
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material3.Checkbox
import androidx.compose.material3.CircularProgressIndicator
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.Slider
import androidx.compose.material3.Switch
import androidx.compose.material3.SwitchDefaults
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableFloatStateOf
import androidx.compose.runtime.mutableStateMapOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.core.Protocol
import com.zitto.vcumaster.core.dbl
import com.zitto.vcumaster.core.long
import com.zitto.vcumaster.core.str
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.Badge
import com.zitto.vcumaster.ui.components.BadgeKind
import com.zitto.vcumaster.ui.components.ConfirmDialog
import com.zitto.vcumaster.ui.components.ConsoleBox
import com.zitto.vcumaster.ui.components.KvList
import com.zitto.vcumaster.ui.components.NumberField
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.PrimaryButton
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.components.SmallButton
import com.zitto.vcumaster.ui.components.rememberNow
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono

private data class ModMeta(val name: String, val desc: String, val key: String)

private val MODS = listOf(
    ModMeta("IMU", "ICM-42670-P over I2C", "imu"),
    ModMeta("CSA", "Current sense", "csa"),
    ModMeta("CAN1", "FlexCAN1, TCAN334", "can1"),
    ModMeta("CAN2", "FlexCAN2, PTC16/PTB13", "can2"),
    ModMeta("FLM", "Flash log manager", "flm"),
)

@Composable
fun DeviceScreen(st: HubState, hub: Hub, prefs: UiPrefs) {
    var side by remember { mutableStateOf("s32") }
    ScreenColumn {
        Seg(listOf("s32" to "S32K144 VCU", "esp" to "ESP32-S3 bridge"), side, { side = it }, Modifier.fillMaxWidth())
        if (side == "s32") S32Side(st, hub, prefs) else EspSide(st, hub)
    }
}

@Composable
private fun S32Side(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val now by rememberNow(500)
    val L = st.latest
    val s = L.status
    val hb = L.heartbeat
    var confirmReset by remember { mutableStateOf(false) }
    var confirmDel by remember { mutableStateOf(false) }

    CommandLogPanel(st)

    Panel("System status", actions = {
        SmallButton("Request") { hub.statusReq() }
        SmallButton("Reset MCU", danger = true) { confirmReset = true }
    }) {
        KvList(
            listOf(
                "Uptime" to (s?.let { Fmt.dur(it.dbl("uptime")) } ?: hb?.let { Fmt.dur(it.dbl("uptime")) } ?: "–"),
                "Heartbeats" to (s?.str("hb") ?: "–"),
                "Reset cause" to (s?.let { "${it.str("reset_name")} (0x${it.long("reset").toString(16).uppercase()})" } ?: "–"),
                "OTA pending" to (s?.let { if (it.long("ota") != 0L) "Yes" else "No" } ?: "–"),
                "CAN1 baud" to (s?.let { "${it.str("can1_baud")} kbps" } ?: "–"),
                "CAN2 baud" to (s?.let { "${it.str("can2_baud")} kbps" } ?: "–"),
                "Flash free" to (s?.let { "${it.str("flash_free")} pages" } ?: "–"),
                "Last status" to Fmt.ago(L.statusT),
            ),
            columns = 2,
        )
        Spacer(Modifier.height(6.dp))
    }

    Panel("Modules", sub = "CMD_MODULE_EN") {
        for (m in MODS) {
            val c = v.channel(m.name.replace("FLM", "FLASH"))
            val on = s?.long(m.key) == 1L
            val p = st.pend["mod:${m.name}"]?.state
            val pend = p == "pending"
            Row(
                Modifier.fillMaxWidth().height(IntrinsicSize.Min).padding(horizontal = 8.dp, vertical = 2.dp)
                    .then(cmdGlow(p)).padding(horizontal = 4.dp, vertical = 1.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Box(Modifier.width(4.dp).fillMaxHeight().background(c, RoundedCornerShape(2.dp)))
                Spacer(Modifier.width(10.dp))
                Column(Modifier.weight(1f)) {
                    Text(m.name, fontWeight = FontWeight.SemiBold, fontSize = 14.sp)
                    Text(m.desc, fontSize = 12.sp, color = v.ink3)
                }
                if (pend) CircularProgressIndicator(Modifier.padding(end = 8.dp).width(18.dp).height(18.dp), strokeWidth = 2.dp)
                Switch(
                    checked = on,
                    onCheckedChange = { want -> hub.module(m.name, want) },
                    colors = SwitchDefaults.colors(checkedTrackColor = v.ok, checkedThumbColor = Color.White),
                )
            }
        }
        if (s == null) Text("  Module state shows once a STATUS frame arrives. Tap Request.", fontSize = 12.sp, color = v.ink3, modifier = Modifier.padding(10.dp))
        Spacer(Modifier.height(6.dp))
    }

    Panel("CAN controllers") {
        Row(Modifier.fillMaxWidth().padding(horizontal = 10.dp, vertical = 4.dp), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
            for (bus in listOf(1, 2)) {
                val c = if (bus == 1) L.can1 else L.can2
                val cc = if (bus == 1) v.can1 else v.can2
                Column(
                    Modifier.weight(1f).border(1.dp, cc.copy(alpha = 0.5f), RoundedCornerShape(10.dp)).padding(10.dp),
                ) {
                    Row(verticalAlignment = Alignment.CenterVertically) {
                        Text("CAN$bus", fontWeight = FontWeight.SemiBold, color = cc, modifier = Modifier.weight(1f))
                        if (c != null) {
                            val bad = c.long("bus_off") != 0L || c.str("state_name") == "ERROR"
                            Badge(c.str("state_name") ?: "?", if (bad) BadgeKind.ERR else if (c.long("ready") != 0L) BadgeKind.OK else BadgeKind.RUN)
                        }
                    }
                    if (c == null) {
                        Text("No status yet", fontSize = 12.sp, color = v.ink3)
                    } else {
                        val rows = listOf(
                            "Baud" to "${c.str("baud")} kbps",
                            "Frames" to (c.str("rx") ?: "–"),
                            "Errors" to "${c.str("err")}" + if (bus == 1) ", TEC ${c.str("tx_err")}, REC ${c.str("rx_err")}" else "",
                            "Bus off" to if (c.long("bus_off") != 0L) "Yes" else "No",
                            "IRQ" to "${c.str("irq")} (err ${c.str("err_irq")}, mb ${c.str("mb_irq")})",
                        )
                        for ((k, x) in rows) {
                            Text(k, fontSize = 11.sp, color = v.ink3, modifier = Modifier.padding(top = 3.dp))
                            Text(x, fontSize = 12.5.sp, fontFamily = Mono)
                        }
                    }
                }
            }
        }
        Spacer(Modifier.height(6.dp))
    }

    GpioPanel(st, hub, prefs)
    LedPanel(hub)

    // ---------------------------------------------------------------- flash
    var flData by remember { mutableStateOf("") }
    var flHex by remember { mutableStateOf(false) }
    Panel("Flash log", actions = { SmallButton("Read latest") { hub.flashRead() } }) {
        val F = L.flm
        KvList(
            if (F != null) listOf(
                "Pages used" to "${F.str("used")} of ${F.str("total")}",
                "Free" to (F.str("free") ?: "–"),
                "Records" to (F.str("records") ?: "–"),
                "Next page" to (F.str("next") ?: "–"),
            ) else listOf("Status" to "No FLM report yet"),
            columns = 2,
        )
        OutlinedTextField(
            flData, { flData = it }, singleLine = true,
            placeholder = { Text(if (flHex) "Hex like 01 A2 FF" else "Record text", fontSize = 12.sp) },
            textStyle = androidx.compose.ui.text.TextStyle(fontFamily = Mono, fontSize = 13.sp, color = v.ink),
            modifier = Modifier.fillMaxWidth().padding(horizontal = 12.dp),
        )
        Row(Modifier.padding(horizontal = 4.dp), verticalAlignment = Alignment.CenterVertically) {
            Checkbox(flHex, { flHex = it })
            Text("Hex", fontSize = 13.sp, modifier = Modifier.weight(1f))
            PrimaryButton("Write") {
                if (flData.isBlank()) hub.toast("Enter something to write first.", true) else hub.flashWrite(flData, flHex)
            }
            Spacer(Modifier.width(6.dp))
            SmallButton("Delete latest", danger = true) { confirmDel = true }
            Spacer(Modifier.width(8.dp))
        }
        ConsoleBox(st.flashOut, 130.dp, "Flash replies (FLASH:OK, FLASH_DATA …) appear here.")
        Spacer(Modifier.height(8.dp))
    }

    if (confirmReset) ConfirmDialog(
        "Reset MCU", "Reset the S32K144? CAN, IMU and CSA streams stop for a moment while it reboots.", "Reset", danger = true,
        onConfirm = { hub.mcuReset() }, onDismiss = { confirmReset = false },
    )
    if (confirmDel) ConfirmDialog(
        "Delete record", "Delete the latest flash record?", "Delete", danger = true,
        onConfirm = { hub.flashDelete() }, onDismiss = { confirmDel = false },
    )
}

@Composable
private fun GpioPanel(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val draft = remember { mutableStateMapOf<Int, Pair<Int, Int>>() }   // id -> dir, state
    var viaMenu by remember { mutableStateOf(false) }
    Panel("GPIO", sub = "S32K144 IDs 1–13, CMD_GPIO_SET", actions = {
        Box {
            TextButton(onClick = { viaMenu = true }) {
                Text(if (prefs.gpioRaw) "via RAW" else "via S32:", fontSize = 12.sp)
                Icon(Icons.Filled.ExpandMore, null)
            }
            DropdownMenu(expanded = viaMenu, onDismissRequest = { viaMenu = false }) {
                DropdownMenuItem(text = { Text("RAW frame (IDs 1–13)") }, onClick = { prefs.setGpioVia(true); viaMenu = false })
                DropdownMenuItem(text = { Text("S32: command (stock bridge)") }, onClick = { prefs.setGpioVia(false); viaMenu = false })
            }
        }
    }) {
        for ((id, port) in Protocol.S32_GPIO_MAP) {
            val d = draft[id] ?: (0 to 0)
            val rep = st.latest.gpio[id]
            HorizontalDivider(color = v.rule2)
            Row(
                Modifier.fillMaxWidth().then(cmdGlow(st.pend["gpio:$id"]?.state)).padding(start = 12.dp, end = 8.dp, top = 6.dp, bottom = 6.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Column(Modifier.width(78.dp)) {
                    Text("#$id  $port", fontFamily = Mono, fontSize = 13.sp, fontWeight = FontWeight.Medium)
                    Text("pin ${Protocol.S32_GPIO_PKG_PIN[id]}", fontSize = 11.sp, color = v.ink3)
                }
                Seg(listOf(0 to "In", 1 to "Out"), d.first, { draft[id] = it to (if (it == 0) 0 else d.second) }, Modifier.width(112.dp))
                Spacer(Modifier.width(6.dp))
                OutlinedButton(
                    onClick = { draft[id] = d.first to (d.second xor 1) },
                    enabled = d.first == 1,
                    contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 8.dp),
                    modifier = Modifier.width(62.dp).height(34.dp),
                ) {
                    Text(if (d.second == 1) "HIGH" else "LOW", fontSize = 11.5.sp, fontFamily = Mono, color = if (d.second == 1) v.ok else v.ink2)
                }
                Column(Modifier.weight(1f).padding(start = 6.dp), horizontalAlignment = Alignment.End) {
                    if (rep != null) {
                        val mismatch = rep.dir == "OUT" && d.first == 1 && rep.state != d.second
                        Text(rep.dir, fontSize = 10.5.sp, color = v.ink3, fontFamily = Mono)
                        Text(
                            if (rep.state == 1) "HIGH" else "LOW", fontSize = 12.sp, fontFamily = Mono,
                            color = if (mismatch) v.warn else if (rep.state == 1) v.ok else v.ink2, fontWeight = FontWeight.Medium,
                        )
                    } else Text("–", color = v.ink3)
                    TextButton(onClick = {
                        if (!prefs.gpioRaw && id == 13) hub.toast("The stock bridge rejects ID 13. Switch to RAW frame or flash the patched bridge.", true)
                        else hub.s32Gpio(id, d.first, if (d.first == 1) d.second else 0, prefs.gpioRaw)
                    }, contentPadding = androidx.compose.foundation.layout.PaddingValues(horizontal = 6.dp)) { Text("Apply", fontSize = 12.sp) }
                }
            }
        }
        Text(
            "  Reported state only updates after a GPIO_SET (the firmware has no GPIO read command).",
            fontSize = 11.5.sp, color = v.ink3, modifier = Modifier.padding(10.dp),
        )
    }
}

@Composable
private fun LedPanel(hub: Hub) {
    val v = LocalVcu.current
    var period by remember { mutableStateOf("500") }
    var duty by remember { mutableFloatStateOf(50f) }
    Panel("Status LED", sub = "PTA0 and PTE5, CMD_LED_CTRL") {
        Row(Modifier.fillMaxWidth().padding(horizontal = 12.dp), verticalAlignment = Alignment.CenterVertically) {
            NumberField("Period", period, { period = it }, Modifier.width(130.dp), suffix = "ms")
            Spacer(Modifier.width(12.dp))
            Column(Modifier.weight(1f)) {
                Text("Duty ${duty.toInt()}%", fontSize = 12.5.sp, color = v.ink2)
                Slider(value = duty, onValueChange = { duty = it }, valueRange = 0f..100f)
            }
        }
        PrimaryButton("Apply", Modifier.padding(start = 12.dp, bottom = 10.dp)) {
            hub.led(period.toIntOrNull() ?: 500, duty.toInt())
        }
    }
}

@Composable
private fun EspSide(st: HubState, hub: Hub) {
    val v = LocalVcu.current
    val L = st.latest
    val B = L.bridge ?: emptyMap()
    Panel("Bridge", actions = {
        SmallButton("Info") { hub.sendCmd("INFO") }
        SmallButton("Stats") { hub.sendCmd("STATS") }
        SmallButton("Ping") { hub.sendCmd("PING") }
    }) {
        KvList(
            listOf(
                "Info" to (L.info ?: "–"),
                "UART frames" to (B.str("uart_frames") ?: B.str("frames") ?: "–"),
                "CRC errors" to (B.str("crc_errors") ?: "–"),
                "Bad length" to (B.str("bad_len") ?: "–"),
                "UART bytes" to (B.str("uart_bytes") ?: "–"),
                "Ping" to (L.pingMs?.let { "$it ms" } ?: "–"),
                "BLE MTU" to (st.link.mtu?.toString() ?: "–"),
                "Updated" to Fmt.ago(L.bridgeT),
            ),
        )
        val raw = L.info?.contains("RAW=1") == true
        if (L.info != null) Text(
            if (raw) "  VCU Master bridge detected (RAW=1): all S32K144 commands are available."
            else "  Stock bridge: only GPIO (via S32:), ESP32 GPIO, PING, INFO, GPIO and STATS work. Flash uart_ble_bridge_vcumaster.ino.",
            fontSize = 12.sp, color = if (raw) v.ok else v.warn, modifier = Modifier.padding(horizontal = 10.dp, vertical = 6.dp),
        )
        Spacer(Modifier.height(6.dp))
    }

    Panel("ESP32 GPIO", sub = "GPIO4 and GPIO5 are reserved for UART2") {
        for (row in Protocol.ESP_ALLOWED_PINS.chunked(3)) {
            Row(Modifier.fillMaxWidth().padding(horizontal = 8.dp, vertical = 3.dp), horizontalArrangement = Arrangement.spacedBy(6.dp)) {
                for (pin in row) {
                    val stv = L.espGpio[pin]
                    Column(
                        Modifier.weight(1f).border(1.dp, if (stv == 1) v.ok.copy(alpha = 0.6f) else v.rule2, RoundedCornerShape(8.dp)).padding(6.dp),
                    ) {
                        Text("GPIO$pin", fontFamily = Mono, fontSize = 12.5.sp, fontWeight = FontWeight.Medium)
                        Text(Protocol.ESP_PIN_NOTES[pin] ?: " ", fontSize = 10.sp, color = v.ink3, maxLines = 1)
                        Row(horizontalArrangement = Arrangement.spacedBy(4.dp)) {
                            LevelButton("LOW", stv == 0, v.ink2, Modifier.weight(1f)) { hub.espGpio(pin, 0) }
                            LevelButton("HIGH", stv == 1, v.ok, Modifier.weight(1f)) { hub.espGpio(pin, 1) }
                        }
                    }
                }
                repeat(3 - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
        Spacer(Modifier.height(8.dp))
    }

    Panel("Protocol reference") {
        Box(Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()).padding(horizontal = 12.dp, vertical = 8.dp)) {
            Text(Protocol.REFERENCE, fontFamily = Mono, fontSize = 11.sp, lineHeight = 15.sp, color = v.ink2)
        }
    }
}

@Composable
private fun LevelButton(text: String, active: Boolean, color: Color, modifier: Modifier, onClick: () -> Unit) {
    val v = LocalVcu.current
    OutlinedButton(
        onClick = onClick, modifier = modifier.height(30.dp),
        contentPadding = androidx.compose.foundation.layout.PaddingValues(0.dp),
        colors = androidx.compose.material3.ButtonDefaults.outlinedButtonColors(
            containerColor = if (active) color.copy(alpha = 0.15f) else Color.Transparent,
        ),
        border = androidx.compose.foundation.BorderStroke(1.dp, if (active) color else v.rule),
    ) { Text(text, fontSize = 10.5.sp, fontFamily = Mono, color = if (active) color else v.ink2) }
}

/** Command feedback colours: glow while waiting, green on the S32K ACK, red on failure / no ACK. */
@Composable
private fun cmdGlow(state: String?): Modifier {
    val v = LocalVcu.current
    val c = when (state) {
        "pending" -> v.focus
        "confirmed" -> v.ok
        "failed", "timeout" -> v.err
        else -> return Modifier
    }
    return Modifier.background(c.copy(alpha = 0.10f), RoundedCornerShape(8.dp)).border(1.dp, c.copy(alpha = 0.7f), RoundedCornerShape(8.dp))
}

@Composable
private fun CommandLogPanel(st: HubState) {
    val v = LocalVcu.current
    Panel("Command log", sub = "sent → S32K log → ACK (green = confirmed by the S32K)") {
        if (st.cmdLog.isEmpty()) {
            Text("  Module, GPIO and LED commands and the S32K's replies appear here.", fontSize = 12.sp, color = v.ink3, modifier = Modifier.padding(10.dp))
        }
        Column(Modifier.fillMaxWidth().heightIn(max = 220.dp).verticalScroll(rememberScrollState()).padding(horizontal = 12.dp)) {
            for (e in st.cmdLog) {
                val c = when (e.cls) { "ok" -> v.ok; "err" -> v.err; "pend" -> v.focus; else -> v.ink2 }
                Row(Modifier.padding(vertical = 2.dp)) {
                    Text(e.ts, fontFamily = Mono, fontSize = 10.5.sp, color = v.ink3, modifier = Modifier.width(92.dp))
                    Text(e.text, fontFamily = Mono, fontSize = 11.5.sp, lineHeight = 15.sp, color = c)
                }
            }
        }
        Spacer(Modifier.height(8.dp))
    }
}
