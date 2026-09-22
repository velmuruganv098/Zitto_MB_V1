package com.zitto.vcumaster.ui

import android.Manifest
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothManager
import android.content.Context
import android.content.Intent
import android.content.pm.PackageManager
import android.os.Build
import androidx.activity.compose.rememberLauncherForActivityResult
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.Arrangement
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.fillMaxSize
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.heightIn
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.statusBarsPadding
import androidx.compose.foundation.layout.width
import androidx.compose.foundation.lazy.LazyColumn
import androidx.compose.foundation.lazy.items
import androidx.compose.material.icons.Icons
import androidx.compose.material.icons.automirrored.filled.List
import androidx.compose.material.icons.filled.Bluetooth
import androidx.compose.material.icons.filled.Delete
import androidx.compose.material.icons.filled.DirectionsCar
import androidx.compose.material.icons.filled.FiberManualRecord
import androidx.compose.material.icons.filled.Memory
import androidx.compose.material.icons.filled.MoreVert
import androidx.compose.material.icons.filled.Sensors
import androidx.compose.material.icons.filled.Share
import androidx.compose.material.icons.filled.SystemUpdateAlt
import androidx.compose.material3.AlertDialog
import androidx.compose.material3.DropdownMenu
import androidx.compose.material3.DropdownMenuItem
import androidx.compose.material3.HorizontalDivider
import androidx.compose.material3.Icon
import androidx.compose.material3.IconButton
import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.NavigationBar
import androidx.compose.material3.NavigationBarItem
import androidx.compose.material3.NavigationBarItemDefaults
import androidx.compose.material3.Scaffold
import androidx.compose.material3.Snackbar
import androidx.compose.material3.SnackbarDuration
import androidx.compose.material3.SnackbarHost
import androidx.compose.material3.SnackbarHostState
import androidx.compose.material3.SnackbarVisuals
import androidx.compose.material3.Text
import androidx.compose.material3.TextButton
import androidx.compose.runtime.Composable
import androidx.compose.runtime.DisposableEffect
import androidx.compose.runtime.LaunchedEffect
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.remember
import androidx.compose.runtime.setValue
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.vector.ImageVector
import androidx.compose.ui.platform.LocalContext
import androidx.compose.ui.platform.LocalView
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import androidx.core.content.FileProvider
import androidx.lifecycle.compose.collectAsStateWithLifecycle
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.ui.components.Pill
import com.zitto.vcumaster.ui.components.Ribbon
import com.zitto.vcumaster.ui.components.rememberNow
import com.zitto.vcumaster.ui.screens.ConnectScreen
import com.zitto.vcumaster.ui.screens.DeviceScreen
import com.zitto.vcumaster.ui.screens.LiveScreen
import com.zitto.vcumaster.ui.screens.SensorsScreen
import com.zitto.vcumaster.ui.screens.UpdatesScreen
import com.zitto.vcumaster.ui.screens.VehicleScreen
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono
import kotlinx.coroutines.launch
import java.io.File

enum class Tab(val key: String, val label: String, val title: String, val icon: ImageVector) {
    CONNECT("connect", "Link", "Connection", Icons.Filled.Bluetooth),
    LIVE("live", "Live", "Live data", Icons.AutoMirrored.Filled.List),
    SENSORS("system", "Sensors", "IMU and current", Icons.Filled.Sensors),
    VEHICLE("vehicle", "Vehicle", "Vehicle", Icons.Filled.DirectionsCar),
    UPDATES("updates", "OTA/DBC", "OTA and DBC", Icons.Filled.SystemUpdateAlt),
    DEVICE("device", "Device", "ESP32 and S32K", Icons.Filled.Memory),
}

private class ToastVisuals(
    override val message: String,
    val err: Boolean,
) : SnackbarVisuals {
    override val actionLabel: String? = null
    override val withDismissAction = false
    override val duration = if (err) SnackbarDuration.Long else SnackbarDuration.Short
}

/** Share a file through the Android share sheet (save to Files, Drive, mail, ...). */
fun shareFile(ctx: Context, file: File, mime: String) {
    val uri = FileProvider.getUriForFile(ctx, ctx.packageName + ".files", file)
    val i = Intent(Intent.ACTION_SEND).setType(mime)
        .putExtra(Intent.EXTRA_STREAM, uri)
        .putExtra(Intent.EXTRA_SUBJECT, file.name)
        .addFlags(Intent.FLAG_GRANT_READ_URI_PERMISSION)
    ctx.startActivity(Intent.createChooser(i, file.name).addFlags(Intent.FLAG_ACTIVITY_NEW_TASK))
}

private fun blePermissions(): Array<String> =
    if (Build.VERSION.SDK_INT >= 31) arrayOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
    else arrayOf(Manifest.permission.ACCESS_FINE_LOCATION)

/**
 * Returns a gate: gate { action } asks for Bluetooth permissions (and to switch Bluetooth on)
 * if needed, then runs the action.
 */
@Composable
fun rememberBleGate(hub: Hub): ((() -> Unit) -> Unit) {
    val ctx = LocalContext.current
    var pending by remember { mutableStateOf<(() -> Unit)?>(null) }

    fun btOn(): Boolean = ctx.getSystemService(BluetoothManager::class.java)?.adapter?.isEnabled == true

    val enableLauncher = rememberLauncherForActivityResult(ActivityResultContracts.StartActivityForResult()) {
        val p = pending
        pending = null
        if (btOn()) p?.invoke() else hub.toast("Bluetooth is still off. Switch it on to scan.", true)
    }
    val permLauncher = rememberLauncherForActivityResult(ActivityResultContracts.RequestMultiplePermissions()) { res ->
        if (res.values.all { it }) {
            if (btOn()) {
                pending?.invoke(); pending = null
            } else {
                try {
                    enableLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
                } catch (_: SecurityException) {
                    hub.toast("Switch Bluetooth on in Android settings.", true)
                }
            }
        } else {
            pending = null
            hub.toast("Bluetooth permission is needed to scan and connect. Allow Nearby devices in app settings.", true)
        }
    }
    return { action ->
        val missing = blePermissions().filter { ContextCompat.checkSelfPermission(ctx, it) != PackageManager.PERMISSION_GRANTED }
        pending = action
        if (missing.isNotEmpty()) permLauncher.launch(missing.toTypedArray())
        else if (!btOn()) {
            try {
                enableLauncher.launch(Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE))
            } catch (_: SecurityException) {
                hub.toast("Switch Bluetooth on in Android settings.", true)
            }
        } else {
            pending = null
            action()
        }
    }
}

@Composable
fun AppRoot(hub: Hub, prefs: UiPrefs) {
    val st by hub.state.collectAsStateWithLifecycle()
    val v = LocalVcu.current
    val snack = remember { SnackbarHostState() }
    val tab = Tab.entries.firstOrNull { it.key == prefs.tab } ?: Tab.CONNECT
    val gate = rememberBleGate(hub)

    LaunchedEffect(Unit) {
        hub.toasts.collect { t ->
            snack.currentSnackbarData?.dismiss()
            launch { snack.showSnackbar(ToastVisuals(t.msg, t.err)) }
        }
    }

    val view = LocalView.current
    val awake = prefs.keepAwake && (st.link.connected || st.ota.running)
    DisposableEffect(awake) {
        view.keepScreenOn = awake
        onDispose { view.keepScreenOn = false }
    }

    Scaffold(
        containerColor = v.paper,
        topBar = { TopBar(tab, st, hub, prefs) },
        bottomBar = {
            NavigationBar(containerColor = v.panel, tonalElevation = 0.dp) {
                for (t in Tab.entries) {
                    NavigationBarItem(
                        selected = t == tab,
                        onClick = { prefs.setTabName(t.key) },
                        icon = {
                            Box {
                                Icon(t.icon, contentDescription = t.label)
                                if (t == Tab.CONNECT) {
                                    Box(
                                        Modifier.align(Alignment.TopEnd).padding(0.dp)
                                            .background(if (st.link.connected) v.ok else Color.Transparent, MaterialTheme.shapes.small)
                                            .width(7.dp).heightIn(min = 7.dp, max = 7.dp),
                                    )
                                }
                            }
                        },
                        label = { Text(t.label, fontSize = 11.sp, maxLines = 1) },
                        alwaysShowLabel = true,
                        colors = NavigationBarItemDefaults.colors(
                            selectedIconColor = v.ink, selectedTextColor = v.ink,
                            indicatorColor = if (v.dark) Color(0xFF243140) else Color(0xFFE3E9EF),
                            unselectedIconColor = v.ink3, unselectedTextColor = v.ink3,
                        ),
                    )
                }
            }
        },
        snackbarHost = {
            SnackbarHost(snack) { data ->
                val err = (data.visuals as? ToastVisuals)?.err == true
                Snackbar(
                    containerColor = if (err) v.err else v.ink,
                    contentColor = if (err) Color.White else v.panel,
                ) { Text(data.visuals.message) }
            }
        },
    ) { pad ->
        Box(Modifier.fillMaxSize().padding(pad)) {
            when (tab) {
                Tab.CONNECT -> ConnectScreen(st, hub, prefs, gate)
                Tab.LIVE -> LiveScreen(st, hub, prefs)
                Tab.SENSORS -> SensorsScreen(st, prefs)
                Tab.VEHICLE -> VehicleScreen(st, hub, prefs)
                Tab.UPDATES -> UpdatesScreen(st, hub, prefs)
                Tab.DEVICE -> DeviceScreen(st, hub, prefs)
            }
        }
    }
}

@Composable
private fun TopBar(tab: Tab, st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val now by rememberNow(100)
    var menu by remember { mutableStateOf(false) }
    var logs by remember { mutableStateOf(false) }
    var about by remember { mutableStateOf(false) }
    val L = st.link
    val (pillText, dot, border) = when {
        L.connected && L.kind == "sim" -> Triple("Simulator", v.warn, v.warn)
        L.connected -> Triple("Connected", v.ok, v.ok)
        L.connecting -> Triple("Connecting", v.focus, v.focus)
        L.error != null -> Triple("Link error", v.err, v.rule)
        else -> Triple("Offline", v.ink3, v.rule)
    }
    Column(Modifier.fillMaxWidth().background(v.panel).statusBarsPadding()) {
        Row(Modifier.fillMaxWidth().padding(start = 16.dp, end = 4.dp, top = 6.dp), verticalAlignment = Alignment.CenterVertically) {
            Column(Modifier.weight(1f)) {
                Text(tab.title, style = MaterialTheme.typography.titleLarge, color = v.ink, maxLines = 1)
                Row(verticalAlignment = Alignment.CenterVertically, horizontalArrangement = Arrangement.spacedBy(10.dp)) {
                    Text("${st.rate} msg/s", fontSize = 12.sp, color = v.ink3, fontFamily = Mono)
                    Text("${st.latest.bridge?.get("crc_errors") ?: 0} CRC err", fontSize = 12.sp, color = v.ink3, fontFamily = Mono)
                }
            }
            Pill(pillText, dot, border) { prefs.setTabName(Tab.CONNECT.key) }
            Spacer(Modifier.width(4.dp))
            val rec = st.recording
            TextButton(onClick = { hub.toggleRecord() }) {
                Icon(Icons.Filled.FiberManualRecord, null, tint = if (rec != null) v.err else v.ink3)
                Text(
                    if (rec != null) " ${rec.rows}" else " Rec", fontSize = 12.sp,
                    color = if (rec != null) v.err else v.ink2, fontWeight = FontWeight.Medium,
                )
            }
            Box {
                IconButton(onClick = { menu = true }) { Icon(Icons.Filled.MoreVert, "More") }
                DropdownMenu(expanded = menu, onDismissRequest = { menu = false }) {
                    for ((k, label) in listOf("system" to "Theme: follow system", "light" to "Theme: light", "dark" to "Theme: dark")) {
                        DropdownMenuItem(
                            text = { Text(label + if (prefs.theme == k) "  ✓" else "") },
                            onClick = { prefs.setThemeMode(k); menu = false },
                        )
                    }
                    HorizontalDivider()
                    DropdownMenuItem(
                        text = { Text("Keep screen on while connected" + if (prefs.keepAwake) "  ✓" else "") },
                        onClick = { prefs.setAwake(!prefs.keepAwake); menu = false },
                    )
                    DropdownMenuItem(text = { Text("Session recordings") }, onClick = { logs = true; menu = false })
                    DropdownMenuItem(text = { Text("About VCU Master") }, onClick = { about = true; menu = false })
                }
            }
        }
        Ribbon(st.ribbon, now, Modifier.padding(horizontal = 16.dp, vertical = 4.dp))
        HorizontalDivider(color = v.rule)
    }
    if (logs) LogsDialog(hub) { logs = false }
    if (about) {
        AlertDialog(
            onDismissRequest = { about = false },
            confirmButton = { TextButton(onClick = { about = false }) { Text("Close") } },
            title = { Text("VCU Master ${Hub.APP_VERSION}") },
            text = {
                Text(
                    "BLE bench console for the Zitto_MB_V1 VCU (NXP S32K144 + ESP32-S3 bridge).\n\n" +
                        "S32K144 ─UART2 115200─> ESP32-S3 bridge ─BLE Nordic UART─> this phone.\n\n" +
                        "Flash esp32/uart_ble_bridge_vcumaster.ino on the bridge for RAW commands " +
                        "(modules, status, reset, LED, flash, OTA). INFO should report RAW=1.",
                )
            },
        )
    }
}

@Composable
private fun LogsDialog(hub: Hub, onDismiss: () -> Unit) {
    val ctx = LocalContext.current
    val v = LocalVcu.current
    var files by remember { mutableStateOf(hub.logFiles()) }
    AlertDialog(
        onDismissRequest = onDismiss,
        confirmButton = { TextButton(onClick = onDismiss) { Text("Close") } },
        title = { Text("Session recordings") },
        text = {
            Column {
                Text(
                    "Saved on the phone in Android/data/${ctx.packageName}/files/logs. Share one to copy it off the phone.",
                    fontSize = 12.sp, color = v.ink3,
                )
                if (files.isEmpty()) Text("No recordings yet. Tap Rec in the top bar to start one.", Modifier.padding(top = 12.dp))
                LazyColumn(Modifier.heightIn(max = 360.dp)) {
                    items(files, key = { it.name }) { f ->
                        Row(verticalAlignment = Alignment.CenterVertically, modifier = Modifier.padding(vertical = 2.dp)) {
                            Column(Modifier.weight(1f)) {
                                Text(f.name, fontFamily = Mono, fontSize = 12.sp)
                                Text("${Fmt.thousands(f.length())} bytes", fontSize = 11.sp, color = v.ink3)
                            }
                            IconButton(onClick = { shareFile(ctx, f, "text/csv") }) { Icon(Icons.Filled.Share, "Share") }
                            IconButton(onClick = { f.delete(); files = hub.logFiles() }) { Icon(Icons.Filled.Delete, "Delete") }
                        }
                    }
                }
            }
        },
    )
}
