package com.zitto.vcumaster.ui.components

import androidx.compose.foundation.BorderStroke
import androidx.compose.foundation.background
import androidx.compose.foundation.border
import androidx.compose.foundation.clickable
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.ColumnScope
import androidx.compose.foundation.layout.PaddingValues
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.RowScope
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.size
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.foundation.lazy.rememberLazyListState
import androidx.compose.foundation.shape.CircleShape
import androidx.compose.foundation.shape.RoundedCornerShape
import androidx.compose.foundation.text.KeyboardOptions
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.Button
import androidx.compose.material3.ButtonDefaults
import androidx.compose.material3.ExperimentalMaterial3Api
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.OutlinedButton
import androidx.compose.material3.OutlinedTextField
import androidx.compose.material3.SegmentedButton
import androidx.compose.material3.SegmentedButtonDefaults
import androidx.compose.material3.SingleChoiceSegmentedButtonRow
import androidx.compose.material3.Surface
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.State
import androidx.compose.runtime.produceState
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.input.KeyboardType
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.ConLine
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono
import kotlinx.coroutines.delay

/** Current time in seconds, refreshed every [periodMs]. */
@Composable
fun rememberNow(periodMs: Long = 250): State<Double> = produceState(Fmt.nowS(), periodMs) {
    while (true) {
        value = Fmt.nowS()
        delay(periodMs)
    }
}

@Composable
fun Panel(
    title: String?,
    modifier: Modifier = Modifier,
    sub: String? = null,
    accent: Color? = null,
    actions: @Composable RowScope.() -> Unit = {},
    content: @Composable ColumnScope.() -> Unit,
) {
    val v = LocalVcu.current
    Surface(
        modifier = modifier.fillMaxWidth(),
        shape = RoundedCornerShape(12.dp),
        color = v.panel,
        border = BorderStroke(1.dp, v.rule),
    ) {
        Column {
            if (title != null) {
                Row(
                    Modifier.fillMaxWidth().padding(start = 14.dp, end = 8.dp, top = 10.dp, bottom = 6.dp),
                    verticalAlignment = Alignment.CenterVertically,
                ) {
                    if (accent != null) {
                        Box(Modifier.size(10.dp).background(accent, RoundedCornerShape(3.dp)))
                        Spacer(Modifier.width(8.dp))
                    }
                    Column(Modifier.weight(1f)) {
                        Text(title, style = MaterialTheme.typography.titleMedium, color = v.ink)
                        if (sub != null) Text(sub, fontSize = 12.sp, color = v.ink3, lineHeight = 15.sp)
                    }
                    Row(horizontalArrangement = Arrangement.spacedBy(2.dp), verticalAlignment = Alignment.CenterVertically, content = actions)
                }
            }
            content()
        }
    }
}

@Composable
fun SectionTitle(text: String) {
    Text(
        text, style = MaterialTheme.typography.titleSmall, color = LocalVcu.current.ink2,
        modifier = Modifier.padding(start = 4.dp, top = 6.dp, bottom = 2.dp),
    )
}

/** Definition list: label / value rows, optionally in 2 columns. */
@Composable
fun KvList(pairs: List<Pair<String, String>>, columns: Int = 1, modifier: Modifier = Modifier) {
    val v = LocalVcu.current
    Column(modifier.fillMaxWidth().padding(horizontal = 14.dp, vertical = 4.dp)) {
        for (row in pairs.chunked(columns)) {
            Row(Modifier.fillMaxWidth().padding(vertical = 3.dp)) {
                for ((k, value) in row) {
                    if (columns == 1) {
                        Text(k, fontSize = 12.5.sp, color = v.ink3, modifier = Modifier.width(118.dp))
                        Text(value, fontSize = 13.sp, fontFamily = Mono, color = v.ink, modifier = Modifier.weight(1f))
                    } else {
                        Column(Modifier.weight(1f).padding(end = 6.dp)) {
                            Text(k, fontSize = 11.5.sp, color = v.ink3)
                            Text(value, fontSize = 13.sp, fontFamily = Mono, color = v.ink, maxLines = 2, overflow = TextOverflow.Ellipsis)
                        }
                    }
                }
                if (columns > 1) repeat(columns - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
    }
}

data class TileData(val label: String, val value: String, val unit: String = "", val stale: Boolean = false)

@Composable
fun TileGrid(tiles: List<TileData>, columns: Int = 3) {
    val v = LocalVcu.current
    Column(Modifier.fillMaxWidth().padding(horizontal = 8.dp, vertical = 4.dp)) {
        for (row in tiles.chunked(columns)) {
            Row(Modifier.fillMaxWidth()) {
                for (t in row) {
                    Column(
                        Modifier.weight(1f).padding(3.dp)
                            .background(if (v.dark) v.rule2 else Color(0xFFF6F8F9), RoundedCornerShape(8.dp))
                            .padding(horizontal = 10.dp, vertical = 8.dp),
                    ) {
                        Text(t.label, fontSize = 11.5.sp, color = v.ink3, maxLines = 1, overflow = TextOverflow.Ellipsis)
                        Row(verticalAlignment = Alignment.Bottom) {
                            Text(
                                t.value, fontFamily = Mono, fontSize = 18.sp, fontWeight = FontWeight.Medium,
                                color = if (t.stale) v.ink3 else v.ink, maxLines = 1, overflow = TextOverflow.Ellipsis,
                                modifier = Modifier.weight(1f, fill = false),
                            )
                            if (t.unit.isNotEmpty()) Text(" " + t.unit, fontSize = 11.sp, color = v.ink3, maxLines = 1)
                        }
                    }
                }
                repeat(columns - row.size) { Spacer(Modifier.weight(1f)) }
            }
        }
    }
}

@OptIn(ExperimentalMaterial3Api::class)
@Composable
fun <T> Seg(options: List<Pair<T, String>>, selected: T, onSelect: (T) -> Unit, modifier: Modifier = Modifier) {
    SingleChoiceSegmentedButtonRow(modifier) {
        options.forEachIndexed { i, (k, label) ->
            SegmentedButton(
                selected = k == selected,
                onClick = { onSelect(k) },
                shape = SegmentedButtonDefaults.itemShape(i, options.size),
                icon = {},
                label = { Text(label, fontSize = 12.sp, maxLines = 1) },
            )
        }
    }
}

enum class BadgeKind { NONE, OK, ERR, RUN, WARN }

@Composable
fun Badge(text: String, kind: BadgeKind = BadgeKind.NONE) {
    val v = LocalVcu.current
    val c = when (kind) {
        BadgeKind.OK -> v.ok; BadgeKind.ERR -> v.err; BadgeKind.RUN -> v.focus; BadgeKind.WARN -> v.warn
        BadgeKind.NONE -> v.ink2
    }
    Text(
        text, fontSize = 12.sp, color = c, maxLines = 1,
        modifier = Modifier
            .background(if (kind == BadgeKind.NONE) v.rule2 else c.copy(alpha = 0.16f), RoundedCornerShape(99.dp))
            .padding(horizontal = 9.dp, vertical = 2.dp),
    )
}

@Composable
fun Dot(color: Color, size: Dp = 8.dp) {
    Box(Modifier.size(size).background(color, CircleShape))
}

@Composable
fun Pill(text: String, dotColor: Color, borderColor: Color, onClick: (() -> Unit)? = null) {
    val v = LocalVcu.current
    Row(
        Modifier
            .border(1.dp, borderColor, RoundedCornerShape(99.dp))
            .then(if (onClick != null) Modifier.clickable { onClick() } else Modifier)
            .padding(horizontal = 9.dp, vertical = 3.dp),
        verticalAlignment = Alignment.CenterVertically,
    ) {
        Dot(dotColor)
        Spacer(Modifier.width(6.dp))
        Text(text, fontSize = 12.sp, color = v.ink, maxLines = 1)
    }
}

/** Scrolling monospace console with tx / rx / er colouring, follows the tail. */
@Composable
fun ConsoleBox(lines: List<ConLine>, height: Dp, empty: String = "") {
    val v = LocalVcu.current
    val st = rememberLazyListState()
    LaunchedEffect(lines.size, lines.lastOrNull()?.t) {
        if (lines.isNotEmpty()) st.scrollToItem(lines.size - 1)
    }
    Box(
        Modifier.fillMaxWidth().height(height).padding(horizontal = 10.dp, vertical = 4.dp)
            .background(if (v.dark) Color(0xFF0D1217) else Color(0xFFF6F8F9), RoundedCornerShape(8.dp))
            .border(1.dp, v.rule2, RoundedCornerShape(8.dp)),
    ) {
        if (lines.isEmpty()) {
            Text(empty, fontSize = 12.sp, color = v.ink3, modifier = Modifier.padding(10.dp))
        }
        LazyColumn(state = st, contentPadding = PaddingValues(8.dp)) {
            items(lines) { l ->
                Text(
                    if (l.t > 0) "${Fmt.clock(l.t)}  ${l.text}" else l.text,
                    fontFamily = Mono, fontSize = 11.5.sp, lineHeight = 15.sp,
                    color = when (l.cls) { "tx" -> v.focus; "er" -> v.err; else -> v.ink },
                )
            }
        }
    }
}

@Composable
fun ConfirmDialog(
    title: String,
    text: String,
    confirm: String,
    danger: Boolean = false,
    onConfirm: () -> Unit,
    onDismiss: () -> Unit,
) {
    val v = LocalVcu.current
    AlertDialog(
        onDismissRequest = onDismiss,
        title = { Text(title) },
        text = { Text(text) },
        confirmButton = {
            Button(
                onClick = { onDismiss(); onConfirm() },
                colors = if (danger) ButtonDefaults.buttonColors(containerColor = v.err, contentColor = Color.White) else ButtonDefaults.buttonColors(),
            ) { Text(confirm) }
        },
        dismissButton = { TextButton(onClick = onDismiss) { Text("Cancel") } },
    )
}

@Composable
fun SmallButton(text: String, modifier: Modifier = Modifier, enabled: Boolean = true, danger: Boolean = false, onClick: () -> Unit) {
    val v = LocalVcu.current
    OutlinedButton(
        onClick = onClick, enabled = enabled, modifier = modifier.heightIn(min = 34.dp),
        contentPadding = PaddingValues(horizontal = 12.dp, vertical = 4.dp),
        border = BorderStroke(1.dp, if (danger) v.err.copy(alpha = 0.6f) else v.rule),
        colors = ButtonDefaults.outlinedButtonColors(contentColor = if (danger) v.err else v.ink),
    ) { Text(text, fontSize = 13.sp, maxLines = 1) }
}

@Composable
fun PrimaryButton(text: String, modifier: Modifier = Modifier, enabled: Boolean = true, onClick: () -> Unit) {
    Button(
        onClick = onClick, enabled = enabled, modifier = modifier.heightIn(min = 38.dp),
        contentPadding = PaddingValues(horizontal = 16.dp, vertical = 6.dp),
    ) { Text(text, fontSize = 13.5.sp, maxLines = 1) }
}

@Composable
fun NumberField(label: String, value: String, onChange: (String) -> Unit, modifier: Modifier = Modifier, suffix: String? = null) {
    val suf: (@Composable () -> Unit)? = suffix?.let { s -> @Composable { Text(s, fontSize = 12.sp) } }
    OutlinedTextField(
        value = value,
        onValueChange = { s -> onChange(s.filter { it.isDigit() }.take(6)) },
        label = { Text(label, fontSize = 12.sp) },
        singleLine = true,
        suffix = suf,
        keyboardOptions = KeyboardOptions(keyboardType = KeyboardType.Number),
        textStyle = MaterialTheme.typography.bodyMedium.copy(fontFamily = Mono),
        modifier = modifier,
    )
}

@Composable
fun EmptyNote(text: String) {
    Text(text, fontSize = 13.sp, color = LocalVcu.current.ink3, modifier = Modifier.padding(horizontal = 14.dp, vertical = 12.dp))
}

@Composable
fun Note(text: String) {
    Text(text, fontSize = 12.sp, lineHeight = 16.sp, color = LocalVcu.current.ink3, modifier = Modifier.padding(horizontal = 14.dp, vertical = 6.dp))
}

@Composable
fun RssiBars(rssi: Int?) {
    val v = LocalVcu.current
    val n = when {
        rssi == null -> 0
        rssi >= -60 -> 4
        rssi >= -70 -> 3
        rssi >= -80 -> 2
        rssi >= -90 -> 1
        else -> 0
    }
    Row(verticalAlignment = Alignment.Bottom) {
        listOf(5, 8, 11, 14).forEachIndexed { i, h ->
            Box(
                Modifier.padding(end = 2.dp).width(4.dp).height(h.dp)
                    .background(if (i < n) v.ok else v.rule, RoundedCornerShape(1.dp)),
            )
        }
        Spacer(Modifier.width(4.dp))
        Text("${rssi ?: "?"} dBm", fontSize = 11.sp, color = v.ink3, fontFamily = Mono)
    }
}
