package com.zitto.vcumaster.ui.screens

import android.content.Context
import android.net.Uri
import android.provider.OpenableColumns
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.ExpandLess
import androidx.compose.material.icons.filled.ExpandMore
import androidx.compose.material.icons.filled.FileOpen
import androidx.compose.material3.Checkbox
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.LinearProgressIndicator
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.ConLine
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.Badge
import com.zitto.vcumaster.ui.components.BadgeKind
import com.zitto.vcumaster.ui.components.ConfirmDialog
import com.zitto.vcumaster.ui.components.ConsoleBox
import com.zitto.vcumaster.ui.components.EmptyNote
import com.zitto.vcumaster.ui.components.KvList
import com.zitto.vcumaster.ui.components.Note
import com.zitto.vcumaster.ui.components.NumberField
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.PrimaryButton
import com.zitto.vcumaster.ui.components.SmallButton
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlin.math.ceil

private fun displayName(ctx: Context, uri: Uri): String {
    ctx.contentResolver.query(uri, arrayOf(OpenableColumns.DISPLAY_NAME), null, null, null)?.use { c ->
        if (c.moveToFirst()) {
            val i = c.getColumnIndex(OpenableColumns.DISPLAY_NAME)
            if (i >= 0) c.getString(i)?.let { return it }
        }
    }
    return uri.lastPathSegment?.substringAfterLast('/') ?: "file"
}

private suspend fun readBytes(ctx: Context, uri: Uri): ByteArray = withContext(Dispatchers.IO) {
    ctx.contentResolver.openInputStream(uri)?.use { it.readBytes() } ?: ByteArray(0)
}

private val OTA_LABEL = mapOf(
    "IDLE" to "Idle", "LOADED" to "Ready", "STARTING" to "Starting", "SENDING" to "Sending",
    "FINISHING" to "Verifying", "COMPLETE" to "Complete", "FAILED" to "Failed", "ABORTED" to "Aborted",
)

@Composable
fun UpdatesScreen(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val ctx = LocalContext.current
    val scope = rememberCoroutineScope()
    val o = st.ota
    var chunk by remember { mutableStateOf(prefs.otaChunk.toString()) }
    var gap by remember { mutableStateOf(prefs.otaDelay.toString()) }
    var confirmStart by remember { mutableStateOf(false) }
    var pendingBin by remember { mutableStateOf<Pair<String, ByteArray>?>(null) }
    var dbcB1 by remember { mutableStateOf(true) }
    var dbcB2 by remember { mutableStateOf(true) }
    var confirmRemove by remember { mutableStateOf<String?>(null) }

    val pickBin = rememberLauncherForActivityResult(ActivityResultContracts.OpenDocument()) { uri ->
        if (uri != null) scope.launch {
            val name = displayName(ctx, uri)
            val data = try { readBytes(ctx, uri) } catch (e: Exception) { hub.toast("File could not be read: ${e.message}", true); return@launch }
            if (!name.lowercase().endsWith(".bin")) pendingBin = name to data else hub.otaLoad(name, data)
        }
    }
    val pickDbc = rememberLauncherForActivityResult(ActivityResultContracts.OpenMultipleDocuments()) { uris ->
        val buses = listOfNotNull(if (dbcB1) 1 else null, if (dbcB2) 2 else null)
        if (uris.isEmpty()) return@rememberLauncherForActivityResult
        if (buses.isEmpty()) { hub.toast("Pick CAN1, CAN2 or both before uploading.", true); return@rememberLauncherForActivityResult }
        scope.launch {
            for (u in uris) {
                val name = displayName(ctx, u)
                val text = try { String(readBytes(ctx, u), Charsets.ISO_8859_1) } catch (e: Exception) { hub.toast("$name: ${e.message}", true); continue }
                hub.loadDbc(name, text, buses)
            }
        }
    }

    ScreenColumn {
        // ------------------------------------------------------------ OTA
        Panel("S32K144 firmware update", actions = {
            Badge(
                OTA_LABEL[o.state] ?: o.state,
                when { o.state == "COMPLETE" -> BadgeKind.OK; o.state == "FAILED" -> BadgeKind.ERR; o.running -> BadgeKind.RUN; else -> BadgeKind.NONE },
            )
            Spacer(Modifier.width(6.dp))
        }) {
            OutlinedButton(
                onClick = { pickBin.launch(arrayOf("application/octet-stream", "*/*")) },
                enabled = !o.running,
                modifier = Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 4.dp).height(56.dp),
                shape = RoundedCornerShape(10.dp),
            ) {
                Icon(Icons.Filled.FileOpen, null)
                Text("  " + (o.name ?: "Choose a .bin image"), maxLines = 1)
            }
            val kv = if (o.name != null) mutableListOf(
                "Image" to o.name,
                "Size" to "${Fmt.thousands(o.size.toLong())} bytes",
                "CRC32" to (o.crc ?: "–"),
                "Sent" to "${Fmt.thousands(o.sent.toLong())} bytes",
                "Rate" to (if (o.rateBps > 0) "${o.rateBps} B/s" else "–"),
                "Time left" to (if (o.etaS != null && o.running) Fmt.dur(o.etaS * 1000.0) else "–"),
            ) else mutableListOf("Image" to "None loaded")
            o.error?.let { kv += "Error" to it }
            KvList(kv)
            Row(Modifier.fillMaxWidth().padding(horizontal = 12.dp), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                NumberField("Chunk", chunk, { chunk = it }, Modifier.weight(1f), suffix = "bytes")
                NumberField("Gap", gap, { gap = it }, Modifier.weight(1f), suffix = "ms")
            }
            val ch = chunk.toIntOrNull() ?: 96
            val gp = gap.toIntOrNull() ?: 0
            if (o.size > 0) {
                val est = ceil(o.size.toDouble() / maxOf(8, ch)) * (gp + 18) / 1000.0
                Note("About ${Fmt.dur(est * 1000)} at these settings. Max chunk at this MTU: ${hub.maxOtaChunk()} bytes.")
            }
            Column(Modifier.padding(horizontal = 14.dp, vertical = 6.dp)) {
                LinearProgressIndicator(
                    progress = { (o.pct / 100).toFloat() },
                    modifier = Modifier.fillMaxWidth().height(8.dp),
                    color = if (o.state == "FAILED") v.err else if (o.state == "COMPLETE") v.ok else v.focus,
                    trackColor = v.rule2,
                )
                Text("${o.pct}%", fontFamily = Mono, fontSize = 12.sp, color = v.ink2, modifier = Modifier.padding(top = 2.dp))
            }
            Row(Modifier.padding(horizontal = 12.dp), horizontalArrangement = Arrangement.spacedBy(8.dp)) {
                PrimaryButton("Start update", enabled = o.name != null && !o.running && st.link.connected) {
                    prefs.setOta(ch, gp)
                    confirmStart = true
                }
                SmallButton("Abort", enabled = o.running, danger = true) { hub.otaAbort() }
            }
            Note(
                "Sequence: CMD_OTA_START (size, CRC32 big-endian) → CMD_OTA_DATA chunks → CMD_OTA_FINISH. " +
                    "The MCU verifies the CRC32 and replies OTA:ok or OTA:verify_fail. Data chunks are not " +
                    "acknowledged by the current firmware, so the gap setting paces the transfer. Start with 96 B / 30 ms.",
            )
            ConsoleBox(o.events.map { ConLine(0.0, it, if ("FAIL" in it.uppercase()) "er" else "rx") }.toList(), 150.dp, "Transfer events appear here.")
            Spacer(Modifier.height(8.dp))
        }

        // ------------------------------------------------------------ DBC
        Panel("CAN databases", actions = { SmallButton("Load demo DBC") { hub.loadSampleDbc() } }) {
            OutlinedButton(
                onClick = { pickDbc.launch(arrayOf("*/*")) },
                modifier = Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 4.dp).height(52.dp),
                shape = RoundedCornerShape(10.dp),
            ) {
                Icon(Icons.Filled.FileOpen, null)
                Text("  Choose .dbc files")
            }
            Row(Modifier.padding(horizontal = 6.dp), verticalAlignment = Alignment.CenterVertically) {
                Text("  Decode on", fontSize = 13.sp, color = v.ink2)
                Checkbox(dbcB1, { dbcB1 = it }); Text("CAN1", fontSize = 13.sp)
                Checkbox(dbcB2, { dbcB2 = it }); Text("CAN2", fontSize = 13.sp)
            }
            if (st.dbcList.isEmpty()) EmptyNote("No DBC loaded. Frames still show in the CAN trace as raw bytes.")
            for (d in st.dbcList) {
                HorizontalDivider(color = v.rule2)
                Row(Modifier.fillMaxWidth().padding(start = 14.dp, end = 6.dp, top = 4.dp, bottom = 4.dp), verticalAlignment = Alignment.CenterVertically) {
                    Column(Modifier.weight(1f)) {
                        Text(d.name, fontWeight = FontWeight.SemiBold, fontSize = 13.5.sp)
                        Text("${d.messages} messages, ${d.signals} signals", fontSize = 12.sp, color = v.ink3)
                        Row(verticalAlignment = Alignment.CenterVertically) {
                            for (b in listOf(1, 2)) {
                                Checkbox(checked = b in d.buses, onCheckedChange = { on ->
                                    val nb = (if (on) d.buses + b else d.buses - b).distinct().sorted()
                                    hub.setDbcBuses(d.name, nb)
                                })
                                Text("CAN$b", fontSize = 12.5.sp)
                            }
                        }
                    }
                    SmallButton("Remove", danger = true) { confirmRemove = d.name }
                }
            }
            Spacer(Modifier.height(8.dp))
        }

        DbcBrowser(st)
    }

    if (confirmStart) ConfirmDialog(
        "Start firmware update",
        "Send ${o.name} (${o.size} bytes, CRC32 ${o.crc}) to the S32K144?\n\nKeep the bench powered and the phone in BLE range until it completes.",
        "Start",
        onConfirm = { hub.otaStart(chunk.toIntOrNull() ?: 96, gap.toIntOrNull() ?: 30) },
        onDismiss = { confirmStart = false },
    )
    pendingBin?.let { (n, data) ->
        ConfirmDialog(
            "Not a .bin file", "$n is not a .bin file. The MCU expects a raw binary image. Load it anyway?", "Load",
            onConfirm = { hub.otaLoad(n, data) }, onDismiss = { pendingBin = null },
        )
    }
    confirmRemove?.let { n ->
        ConfirmDialog(
            "Remove DBC", "Remove $n? Its decoded values are cleared.", "Remove", danger = true,
            onConfirm = { hub.removeDbc(n) }, onDismiss = { confirmRemove = null },
        )
    }
}

@Composable
private fun DbcBrowser(st: HubState) {
    val v = LocalVcu.current
    var pick by remember { mutableStateOf<String?>(null) }
    var menu by remember { mutableStateOf(false) }
    var open by remember { mutableStateOf<String?>(null) }
    val name = pick?.takeIf { it in st.dbcs } ?: st.dbcs.keys.firstOrNull()
    val db = name?.let { st.dbcs[it] }

    Panel("DBC browser", actions = {
        Box {
            TextButton(onClick = { menu = true }, enabled = st.dbcs.isNotEmpty()) {
                Text(name ?: "No DBC loaded", fontSize = 12.5.sp, maxLines = 1)
                Icon(Icons.Filled.ExpandMore, null)
            }
            DropdownMenu(expanded = menu, onDismissRequest = { menu = false }) {
                for (n in st.dbcs.keys) DropdownMenuItem(text = { Text(n) }, onClick = { pick = n; menu = false })
            }
        }
    }) {
        if (db == null) {
            EmptyNote("Load a DBC to browse its messages and signals.")
            return@Panel
        }
        for (m in db.messages) {
            val isOpen = open == m.name
            HorizontalDivider(color = v.rule2)
            Row(
                Modifier.fillMaxWidth().clickable { open = if (isOpen) null else m.name }.padding(horizontal = 14.dp, vertical = 8.dp),
                verticalAlignment = Alignment.CenterVertically,
            ) {
                Column(Modifier.weight(1f)) {
                    Text(m.name, fontWeight = FontWeight.Medium, fontSize = 13.5.sp)
                    Text(
                        "${Fmt.hexId(m.frameId, m.ext)} dlc ${m.length}" +
                            (m.cycleMs?.let { ", $it ms" } ?: "") +
                            (if (m.senders.isNotEmpty()) ", from ${m.senders.joinToString(", ")}" else "") +
                            ", ${m.signals.size} signals",
                        fontFamily = Mono, fontSize = 11.5.sp, color = v.ink3,
                    )
                    if (m.comment.isNotEmpty()) Text(m.comment, fontSize = 11.5.sp, color = v.ink3)
                }
                Icon(if (isOpen) Icons.Filled.ExpandLess else Icons.Filled.ExpandMore, null, tint = v.ink3)
            }
            if (isOpen) {
                for (s in m.signals) {
                    Column(
                        Modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 3.dp)
                            .background(v.rule2.copy(alpha = 0.6f), RoundedCornerShape(8.dp)).padding(horizontal = 10.dp, vertical = 6.dp),
                    ) {
                        Row {
                            Text(s.name, fontWeight = FontWeight.Medium, fontSize = 13.sp, modifier = Modifier.weight(1f))
                            Text(s.unit, fontSize = 12.sp, color = v.ink3)
                        }
                        Text(
                            "start ${s.start}  len ${s.length}  ${if (s.littleEndian) "Intel" else "Motorola"}${if (s.signed) ", signed" else ""}" +
                                (if (s.muxSwitch) "  MUX" else "") + (s.muxId?.let { "  m$it" } ?: ""),
                            fontFamily = Mono, fontSize = 11.sp, color = v.ink2,
                        )
                        Text(
                            "scale ${Fmt.num(s.scale)}  offset ${Fmt.num(s.offset)}  range ${s.min?.let { Fmt.num(it) } ?: ""} to ${s.max?.let { Fmt.num(it) } ?: ""}",
                            fontFamily = Mono, fontSize = 11.sp, color = v.ink2,
                        )
                        if (s.choices.isNotEmpty()) {
                            Text(s.choices.entries.joinToString(", ") { "${it.key}=${it.value}" }, fontSize = 11.5.sp, color = v.focus)
                        }
                        if (s.comment.isNotEmpty()) Text(s.comment, fontSize = 11.sp, color = v.ink3)
                    }
                }
                Spacer(Modifier.height(6.dp))
            }
        }
        Spacer(Modifier.height(8.dp))
    }
}
