package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.horizontalScroll
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.IntrinsicSize
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxHeight
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.rememberScrollState
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.selection.SelectionContainer
import androidx.compose.foundation.verticalScroll
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.filled.Close
import androidx.compose.material.icons.filled.FilterList
import androidx.compose.material.icons.filled.MoreVert
import androidx.compose.material.icons.filled.Pause
import androidx.compose.material.icons.filled.PlayArrow
import androidx.compose.material.icons.filled.VerticalAlignBottom
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Checkbox
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.FilterChip
import androidx.compose.material3.FilterChipDefaults
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.SmallFloatingActionButton
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableIntStateOf
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.rememberCoroutineScope
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.platform.LocalClipboardManager
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.text.AnnotatedString
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.CHIP_ORDER
import com.zitto.vcumaster.core.CH_LABEL
import com.zitto.vcumaster.core.CH_ORDER
import com.zitto.vcumaster.core.CustomFilter
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.core.Rec
import com.zitto.vcumaster.core.recJson
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.ConfirmDialog
import com.zitto.vcumaster.ui.components.Dot
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.shareFile
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext

private class LiveFilter(
    val tags: Set<String>,
    val search: Regex?,
    val ids: Set<Long>?,
    val custom: List<CustomFilter>,
) {
    fun passes(r: Rec): Boolean {
        if (tags.isNotEmpty() && r.tags.none { it in tags }) return false
        if (ids != null && !(r.type == "CAN" && r.canId?.let { it in ids } == true)) return false
        if (search != null && !search.containsMatchIn(r.searchText)) return false
        if (custom.isEmpty()) return true
        return custom.any { f -> (f.tag.isEmpty() || f.tag in r.tags) && f.re.containsMatchIn(r.searchText) }
    }
}

private fun compileSearch(q: String): Regex? {
    val s = q.trim()
    if (s.isEmpty()) return null
    val m = Regex("^/(.+)/(i?)$").matchEntire(s)
    return try {
        if (m != null) Regex(m.groupValues[1], RegexOption.IGNORE_CASE) else Regex(Regex.escape(s), RegexOption.IGNORE_CASE)
    } catch (_: Exception) {
        null
    }
}

private fun parseIds(s: String): Set<Long>? {
    val ids = s.split(Regex("[\\s,]+")).filter { it.isNotBlank() }
        .mapNotNull { it.removePrefix("0x").removePrefix("0X").toLongOrNull(16) }
    return if (ids.isEmpty()) null else ids.toSet()
}

private fun latestKey(r: Rec): String = when (r.type) {
    "CAN" -> "CAN ${r.fields["bus"]} ${(r.canId ?: 0).toString().padStart(10, '0')}"
    "CAN_STATUS" -> "CS ${r.fields["bus"]}"
    else -> r.type
}

@Composable
fun LiveScreen(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val ctx = LocalContext.current
    val scope = rememberCoroutineScope()
    var search by remember { mutableStateOf("") }
    var idsText by remember { mutableStateOf("") }
    var mode by remember { mutableStateOf("stream") }
    var paused by remember { mutableStateOf(false) }
    var rows by remember { mutableStateOf<List<Rec>>(emptyList()) }
    var total by remember { mutableIntStateOf(0) }
    var detail by remember { mutableStateOf<Rec?>(null) }
    var menu by remember { mutableStateOf(false) }
    var showFilters by remember { mutableStateOf(false) }
    var confirmClear by remember { mutableStateOf(false) }
    val list = rememberLazyListState()

    val activeCustom = st.filters.filter { it.name in prefs.cfOn }
    val filter = LiveFilter(prefs.tagSel, compileSearch(search), parseIds(idsText), activeCustom)

    LaunchedEffect(prefs.tagSel, search, idsText, mode, paused, prefs.cfOn, st.filters) {
        while (true) {
            if (!paused) {
                val recs = hub.state.value.records
                val m = mode
                val (r, t) = withContext(Dispatchers.Default) {
                    val all = recs.filter { filter.passes(it) }
                    if (m == "latest") {
                        val map = LinkedHashMap<String, Rec>()
                        for (x in all) map[latestKey(x)] = x
                        map.entries.sortedBy { it.key }.map { it.value } to all.size
                    } else {
                        all.takeLast(700) to all.size
                    }
                }
                val lastVisible = list.layoutInfo.visibleItemsInfo.lastOrNull()?.index ?: -1
                val atBottom = lastVisible >= rows.size - 3
                rows = r
                total = t
                if (m != "latest" && atBottom && r.isNotEmpty()) list.scrollToItem(r.size - 1)
            }
            delay(300)
        }
    }

    fun export(csv: Boolean) {
        scope.launch {
            val recs = hub.state.value.records
            val sel = withContext(Dispatchers.Default) { recs.filter { filter.passes(it) } }
            val name = "vcu_master_${Fmt.fileStamp()}.${if (csv) "csv" else "txt"}"
            val body = withContext(Dispatchers.Default) {
                if (csv) {
                    (listOf("pc_time,seq,channel,type,raw,decoded") + sel.map { r ->
                        listOf(Fmt.clock(r.t), r.seq?.toString() ?: "", r.ch, r.type, r.raw, r.decodeText).joinToString(",") { Fmt.csv(it) }
                    }).joinToString("\n")
                } else {
                    sel.joinToString("\n") { r -> "${Fmt.clock(r.t)} ${r.raw}" + if (r.dbc != null) "   | ${r.decodeText}" else "" }
                }
            }
            val f = hub.writeExport(name, body)
            shareFile(ctx, f, if (csv) "text/csv" else "text/plain")
        }
    }

    Column(Modifier.fillMaxSize()) {
        // ---------------------------------------------------------------- chips
        Row(
            Modifier.fillMaxWidth().horizontalScroll(rememberScrollState()).padding(horizontal = 10.dp, vertical = 6.dp),
            horizontalArrangement = Arrangement.spacedBy(6.dp),
        ) {
            FilterChip(
                selected = prefs.tagSel.isEmpty(), onClick = { prefs.setTags(emptySet()) },
                label = { Text("All", fontSize = 12.sp) },
            )
            for (t in CHIP_ORDER) {
                val c = v.channel(t)
                FilterChip(
                    selected = t in prefs.tagSel,
                    onClick = { prefs.setTags(if (t in prefs.tagSel) prefs.tagSel - t else prefs.tagSel + t) },
                    leadingIcon = { Dot(c, 9.dp) },
                    label = { Text("${CH_LABEL[t]}  ${st.counts["#$t"] ?: 0}", fontSize = 12.sp, maxLines = 1) },
                    colors = FilterChipDefaults.filterChipColors(selectedContainerColor = c.copy(alpha = 0.14f)),
                )
            }
        }
        // ---------------------------------------------------------------- search row
        Row(Modifier.fillMaxWidth().padding(horizontal = 10.dp), verticalAlignment = Alignment.CenterVertically) {
            OutlinedTextField(
                search, { search = it }, singleLine = true, modifier = Modifier.weight(1.4f),
                placeholder = { Text("Search or /regex/", fontSize = 12.sp) },
                textStyle = androidx.compose.ui.text.TextStyle(fontSize = 13.sp, color = v.ink),
                isError = search.isNotBlank() && compileSearch(search) == null,
            )
            Spacer(Modifier.width(6.dp))
            OutlinedTextField(
                idsText, { idsText = it }, singleLine = true, modifier = Modifier.weight(1f),
                placeholder = { Text("CAN IDs 0x101", fontSize = 12.sp, fontFamily = Mono) },
                textStyle = androidx.compose.ui.text.TextStyle(fontSize = 13.sp, fontFamily = Mono, color = v.ink),
            )
        }
        // ---------------------------------------------------------------- mode row
        Row(Modifier.fillMaxWidth().padding(start = 10.dp, end = 2.dp, top = 4.dp), verticalAlignment = Alignment.CenterVertically) {
            Seg(listOf("stream" to "Stream", "latest" to "Latest", "raw" to "Raw"), mode, { mode = it }, Modifier.weight(1f))
            IconButton(onClick = { paused = !paused }) {
                Icon(if (paused) Icons.Filled.PlayArrow else Icons.Filled.Pause, if (paused) "Resume" else "Pause", tint = if (paused) v.warn else v.ink)
            }
            IconButton(onClick = { showFilters = !showFilters }) {
                Icon(Icons.Filled.FilterList, "Custom filters", tint = if (activeCustom.isNotEmpty()) v.focus else v.ink)
            }
            Box {
                IconButton(onClick = { menu = true }) { Icon(Icons.Filled.MoreVert, "More") }
                DropdownMenu(expanded = menu, onDismissRequest = { menu = false }) {
                    DropdownMenuItem(text = { Text("Export CSV (filtered)") }, onClick = { menu = false; export(true) })
                    DropdownMenuItem(text = { Text("Export TXT (filtered)") }, onClick = { menu = false; export(false) })
                    HorizontalDivider()
                    DropdownMenuItem(text = { Text("Clear collected data", color = v.err) }, onClick = { menu = false; confirmClear = true })
                }
            }
        }
        if (showFilters) CustomFilters(st, hub, prefs)
        // ---------------------------------------------------------------- footer text
        Row(Modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 2.dp)) {
            Text(
                if (mode == "latest") "${rows.size} distinct message types from $total matching"
                else "Showing ${rows.size} of $total matching (${st.records.size} collected)",
                fontSize = 11.5.sp, color = v.ink3, modifier = Modifier.weight(1f),
            )
            if (paused) Text("Paused", fontSize = 11.5.sp, color = v.warn)
        }
        // ---------------------------------------------------------------- log
        Box(
            Modifier.weight(1f).fillMaxWidth().padding(horizontal = 8.dp, vertical = 4.dp)
                .background(v.panel, RoundedCornerShape(10.dp)).border(1.dp, v.rule, RoundedCornerShape(10.dp)),
        ) {
            if (rows.isEmpty()) {
                Text(
                    if (st.records.isEmpty()) "No data yet. Connect to a bridge (or the simulator) on the Link tab."
                    else "No messages match these filters. Clear a filter or wait for data.",
                    color = v.ink3, fontSize = 13.sp, modifier = Modifier.padding(16.dp),
                )
            }
            LazyColumn(state = list, contentPadding = PaddingValues(vertical = 4.dp), modifier = Modifier.fillMaxSize()) {
                items(rows, key = { it.id }) { r -> LogRow(r, mode == "raw") { detail = r } }
            }
            if (mode != "latest" && rows.isNotEmpty()) {
                SmallFloatingActionButton(
                    onClick = { scope.launch { list.scrollToItem(rows.size - 1) } },
                    modifier = Modifier.align(Alignment.BottomEnd).padding(10.dp),
                    containerColor = v.ink, contentColor = v.panel,
                ) { Icon(Icons.Filled.VerticalAlignBottom, "Jump to latest") }
            }
        }
    }

    detail?.let { r -> DetailDialog(r) { detail = null } }
    if (confirmClear) ConfirmDialog(
        "Clear data", "Clear all collected data in this session? Recording files are kept.", "Clear", danger = true,
        onConfirm = { hub.clear(); rows = emptyList() }, onDismiss = { confirmClear = false },
    )
}

@Composable
private fun LogRow(r: Rec, raw: Boolean, onClick: () -> Unit) {
    val v = LocalVcu.current
    val c = v.channel(r.ch)
    Row(
        Modifier.fillMaxWidth().height(IntrinsicSize.Min).clickable(onClick = onClick)
            .background(if (r.isErr) v.err.copy(alpha = 0.08f) else Color.Transparent)
            .padding(horizontal = 6.dp, vertical = 3.dp),
    ) {
        Box(Modifier.width(3.dp).fillMaxHeight().heightIn(min = 18.dp).background(c, RoundedCornerShape(2.dp)))
        Spacer(Modifier.width(7.dp))
        Column(Modifier.weight(1f)) {
            if (raw) {
                Text("${Fmt.clock(r.t)}  ${r.raw}", fontFamily = Mono, fontSize = 11.5.sp, lineHeight = 15.sp, color = if (r.isErr) v.err else v.ink)
            } else {
                Row(verticalAlignment = Alignment.CenterVertically) {
                    Text(Fmt.clock(r.t), fontFamily = Mono, fontSize = 11.sp, color = v.ink3)
                    if (r.seq != null) Text("  #${r.seq}", fontFamily = Mono, fontSize = 11.sp, color = v.ink3)
                    Text("  ${CH_LABEL[r.ch]}", fontSize = 11.sp, color = c, fontWeight = FontWeight.SemiBold)
                    Text("  ${r.type}", fontFamily = Mono, fontSize = 11.sp, color = v.ink2, maxLines = 1)
                }
                Text(
                    r.summary, fontFamily = Mono, fontSize = 12.sp, lineHeight = 16.sp,
                    color = if (r.isErr) v.err else v.ink, maxLines = 3, overflow = TextOverflow.Ellipsis,
                )
                if (r.dbc != null) {
                    Text(r.decodeText, fontFamily = Mono, fontSize = 11.5.sp, lineHeight = 15.sp, color = c, maxLines = 3, overflow = TextOverflow.Ellipsis)
                }
            }
        }
    }
}

@Composable
private fun DetailDialog(r: Rec, onDismiss: () -> Unit) {
    val clip = LocalClipboardManager.current
    val json = remember(r.id) { recJson(r) }
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text("Message detail") },
        text = {
            SelectionContainer {
                Text(json, fontFamily = Mono, fontSize = 11.5.sp, lineHeight = 15.sp, modifier = Modifier.heightIn(max = 460.dp).verticalScroll(rememberScrollState()))
            }
        },
        confirmButton = { TextButton(onClick = onDismiss) { Text("Close") } },
        dismissButton = { TextButton(onClick = { clip.setText(AnnotatedString(json)) }) { Text("Copy") } },
    )
}

@Composable
private fun CustomFilters(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    var name by remember { mutableStateOf("") }
    var expr by remember { mutableStateOf("") }
    var isRegex by remember { mutableStateOf(true) }
    var tag by remember { mutableStateOf("") }
    var tagMenu by remember { mutableStateOf(false) }
    Column(
        Modifier.fillMaxWidth().padding(horizontal = 8.dp, vertical = 4.dp)
            .background(v.panel, RoundedCornerShape(10.dp)).border(1.dp, v.rule, RoundedCornerShape(10.dp))
            .padding(8.dp),
    ) {
        Text(
            "Custom filters  (${prefs.cfOn.count { n -> st.filters.any { it.name == n } }} of ${st.filters.size} active, combined with OR)",
            fontSize = 12.5.sp, color = v.ink2,
        )
        if (st.filters.isEmpty()) Text("No custom filters yet. Add one below.", fontSize = 12.sp, color = v.ink3)
        for (f in st.filters) {
            Row(verticalAlignment = Alignment.CenterVertically) {
                Checkbox(checked = f.name in prefs.cfOn, onCheckedChange = { on ->
                    prefs.updateCfOn(if (on) prefs.cfOn + f.name else prefs.cfOn - f.name)
                })
                Column(Modifier.weight(1f)) {
                    Text(f.name, fontSize = 13.sp)
                    Text(f.expr + if (f.tag.isNotEmpty()) "  in ${f.tag}" else "", fontFamily = Mono, fontSize = 11.sp, color = v.ink3, maxLines = 1, overflow = TextOverflow.Ellipsis)
                }
                IconButton(onClick = {
                    prefs.updateCfOn(prefs.cfOn - f.name)
                    hub.removeFilter(f.name)
                }) { Icon(Icons.Filled.Close, "Delete filter", tint = v.ink3) }
            }
        }
        HorizontalDivider(Modifier.padding(vertical = 4.dp), color = v.rule2)
        Row(verticalAlignment = Alignment.CenterVertically) {
            OutlinedTextField(name, { name = it }, singleLine = true, modifier = Modifier.weight(1f), placeholder = { Text("Filter name", fontSize = 12.sp) })
            Spacer(Modifier.width(6.dp))
            Box {
                TextButton(onClick = { tagMenu = true }) { Text(if (tag.isEmpty()) "Any channel" else CH_LABEL[tag] ?: tag, fontSize = 12.sp) }
                DropdownMenu(expanded = tagMenu, onDismissRequest = { tagMenu = false }) {
                    DropdownMenuItem(text = { Text("Any channel") }, onClick = { tag = ""; tagMenu = false })
                    for (t in CH_ORDER) DropdownMenuItem(text = { Text(CH_LABEL[t] ?: t) }, onClick = { tag = t; tagMenu = false })
                }
            }
        }
        OutlinedTextField(
            expr, { expr = it }, singleLine = true, modifier = Modifier.fillMaxWidth(),
            placeholder = { Text("Text or regex, e.g. id=0x1[0-9]{2}|bus_off=1", fontSize = 12.sp, fontFamily = Mono) },
            textStyle = androidx.compose.ui.text.TextStyle(fontFamily = Mono, fontSize = 13.sp, color = v.ink),
        )
        Row(verticalAlignment = Alignment.CenterVertically) {
            Checkbox(isRegex, { isRegex = it })
            Text("Regex", fontSize = 13.sp, modifier = Modifier.weight(1f))
            TextButton(onClick = {
                val n = name.trim()
                val e = expr.trim()
                if (n.isEmpty() || e.isEmpty()) {
                    hub.toast("Give the filter a name and an expression.", true)
                } else {
                    hub.addFilter(CustomFilter(n, e, isRegex, tag))
                    prefs.updateCfOn(prefs.cfOn + n)
                    name = ""; expr = ""
                }
            }) { Text("Add filter") }
        }
    }
}
