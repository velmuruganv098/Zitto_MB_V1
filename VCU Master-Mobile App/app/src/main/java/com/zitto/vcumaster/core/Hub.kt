package com.zitto.vcumaster.core

import android.content.Context
import com.zitto.vcumaster.link.BleLink
import com.zitto.vcumaster.link.BleScanner
import com.zitto.vcumaster.link.GattService
import com.zitto.vcumaster.link.Link
import com.zitto.vcumaster.link.LinkInfo
import com.zitto.vcumaster.link.SIM_ADDRESS
import com.zitto.vcumaster.link.ScanDev
import com.zitto.vcumaster.link.SimLink
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Job
import kotlinx.coroutines.NonCancellable
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.asCoroutineDispatcher
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout
import java.io.BufferedWriter
import java.io.File
import java.util.concurrent.Executors
import kotlin.math.atan2
import kotlin.math.hypot
import kotlin.math.round
import kotlin.math.sqrt

class HubError(msg: String) : Exception(msg)

data class Toast(val msg: String, val err: Boolean = false)

data class ConLine(val t: Double, val text: String, val cls: String)   // cls: tx | rx | er

data class Latest(
    val imu: Map<String, Any?>? = null, val imuT: Double? = null,
    val csa: Map<String, Any?>? = null, val csaT: Double? = null,
    val status: Map<String, Any?>? = null, val statusT: Double? = null,
    val heartbeat: Map<String, Any?>? = null,
    val can1: Map<String, Any?>? = null,
    val can2: Map<String, Any?>? = null,
    val flm: Map<String, Any?>? = null,
    val gpio: Map<Int, GpioPin> = emptyMap(),
    val bridge: Map<String, Any?>? = null, val bridgeT: Double? = null,
    val info: String? = null,
    val espGpio: Map<Int, Int> = emptyMap(),
    val lastAck: Map<String, Any?>? = null,
    val pingMs: Double? = null,
)

data class OtaInfo(
    val state: String = "IDLE", val name: String? = null, val size: Int = 0, val crc: String? = null,
    val sent: Int = 0, val pct: Double = 0.0, val rateBps: Long = 0, val etaS: Long? = null,
    val elapsedS: Double = 0.0, val chunk: Int = 96, val delayMs: Int = 30, val error: String? = null,
    val events: List<String> = emptyList(),
) {
    val running get() = state in RUNNING_STATES
}

private val RUNNING_STATES = setOf("STARTING", "SENDING", "FINISHING")

data class RecInfo(val name: String, val rows: Long, val since: Double)

class SeriesSnap(val t: DoubleArray, val v: DoubleArray) {
    val size get() = t.size
    companion object { val EMPTY = SeriesSnap(DoubleArray(0), DoubleArray(0)) }
}

/** Fixed-size time series ring. */
class Ring(private val cap: Int) {
    private val t = DoubleArray(cap)
    private val v = DoubleArray(cap)
    private var start = 0
    var size = 0; private set

    fun add(tt: Double, vv: Double) {
        val i = (start + size) % cap
        t[i] = tt; v[i] = vv
        if (size < cap) size++ else start = (start + 1) % cap
    }

    fun clear() { start = 0; size = 0 }

    fun snap(): SeriesSnap {
        val ot = DoubleArray(size)
        val ov = DoubleArray(size)
        for (k in 0 until size) {
            val i = (start + k) % cap
            ot[k] = t[i]; ov[k] = v[i]
        }
        return SeriesSnap(ot, ov)
    }

    /** Values newer than t0. */
    fun since(t0: Double): List<Double> {
        val out = ArrayList<Double>()
        for (k in 0 until size) {
            val i = (start + k) % cap
            if (t[i] > t0) out += v[i]
        }
        return out
    }
}

data class ScanState(val scanning: Boolean = false, val devices: List<ScanDev>? = null, val error: String? = null)

data class HubState(
    val link: LinkInfo = LinkInfo(),
    val rate: Double = 0.0,
    val counts: Map<String, Long> = emptyMap(),
    val latest: Latest = Latest(),
    val vehicle: Map<String, VehEntry> = emptyMap(),
    val ota: OtaInfo = OtaInfo(),
    val recording: RecInfo? = null,
    val dbcList: List<DbcSummary> = emptyList(),
    val dbcs: Map<String, DbcDatabase> = emptyMap(),
    val signals: List<SignalStat> = emptyList(),
    val messages: List<MsgStat> = emptyList(),
    val records: List<Rec> = emptyList(),
    val recsVersion: Long = 0,
    val console: List<ConLine> = emptyList(),
    val flashOut: List<ConLine> = emptyList(),
    val charts: Map<String, SeriesSnap> = emptyMap(),
    val plot: Map<String, SeriesSnap> = emptyMap(),
    val plotSel: List<String> = emptyList(),
    val ribbon: List<Pair<Int, Double>> = emptyList(),
    val filters: List<CustomFilter> = emptyList(),
    val vehicleMap: Map<String, String?> = emptyMap(),
    val allSignals: List<String> = emptyList(),
    val services: List<GattService> = emptyList(),
    val lastDevice: Pair<String, String?>? = null,
    val csaAvg10: Double? = null,
    val csaPeak10: Double? = null,
)

/**
 * Runtime state: link, record store, decoders, OTA, recording. All mutation happens on one
 * dedicated thread; the UI observes immutable [HubState] snapshots published every 100 ms.
 */
class Hub(private val ctx: Context) {
    val dispatcher = Executors.newSingleThreadExecutor { r -> Thread(r, "vcu-hub") }.asCoroutineDispatcher()
    val scope = CoroutineScope(SupervisorJob() + dispatcher)

    private val _state = MutableStateFlow(HubState())
    val state: StateFlow<HubState> = _state
    private val _scan = MutableStateFlow(ScanState())
    val scan: StateFlow<ScanState> = _scan
    private val _toasts = MutableSharedFlow<Toast>(extraBufferCapacity = 32)
    val toasts: SharedFlow<Toast> = _toasts

    private val dataDir = File(ctx.filesDir, "data")
    private val dbcDir = File(dataDir, "dbc")
    private val otaDir = File(dataDir, "ota")
    val logDir: File = File(ctx.getExternalFilesDir(null) ?: ctx.filesDir, "logs")
    private val exportDir = File(ctx.cacheDir, "exports")
    private val settings: Settings

    private var link: Link? = null
    private val records = ArrayDeque<Rec>()
    private val maxRecs = 20000
    private var nextId = 1L
    private var recsVersion = 0L
    private var recsSnap: List<Rec> = emptyList()
    private var recsSnapVersion = -1L
    private val counts = HashMap<String, Long>()
    private val rateWindow = ArrayDeque<Double>()
    private var latest = Latest()
    private val dbc = DbcEngine()
    private val waiters = mutableListOf<Pair<Regex, CompletableDeferred<String>>>()
    private var pingT: Double? = null
    private val console = ArrayDeque<ConLine>()
    private val flashOut = ArrayDeque<ConLine>()
    private var flashWait = 0.0
    private val ribbon = ArrayDeque<Pair<Int, Double>>()
    private val rings = linkedMapOf(
        "ax" to Ring(1200), "ay" to Ring(1200), "az" to Ring(1200),
        "gx" to Ring(1200), "gy" to Ring(1200), "gz" to Ring(1200),
        "cur" to Ring(1500), "vol" to Ring(1500), "pow" to Ring(1500),
    )
    private val plotRings = LinkedHashMap<String, Ring>()
    private var plotSel = listOf<String>()
    private var dirty = true

    // recording
    private var recWriter: BufferedWriter? = null
    private var recName: String? = null
    private var recRows = 0L
    private var recSince = 0.0

    // OTA
    private var otaImage: ByteArray? = null
    private var otaName: String? = null
    private var otaCrc: Long? = null
    private var otaState = "IDLE"
    private var otaSent = 0
    private var otaError: String? = null
    private var otaStarted: Double? = null
    private var otaFinished: Double? = null
    private var otaChunk = 96
    private var otaDelay = 30
    private var otaJob: Job? = null
    private val otaEvents = ArrayDeque<String>()

    private val sampleText: String by lazy {
        ctx.assets.open(SAMPLE_DBC).bufferedReader().use { it.readText() }
    }
    private val sampleDb: DbcDatabase? by lazy { try { DbcParser.parse(sampleText) } catch (_: Exception) { null } }

    init {
        for (d in listOf(dataDir, dbcDir, otaDir, logDir, exportDir)) d.mkdirs()
        settings = Settings(File(dataDir, "settings.json"))
        val prefs = ctx.getSharedPreferences("vcum", Context.MODE_PRIVATE)
        plotSel = prefs.getString("plotSel", "")!!.split("\n").filter { it.isNotBlank() }
        scope.launch {
            for ((name, buses) in settings.dbc) {
                val f = File(dbcDir, name)
                if (f.exists()) {
                    try { dbc.load(name, f.readText(), buses) } catch (_: Exception) { }
                }
            }
            for ((role, sig) in settings.vehicleMap) if (role in VEHICLE_ROLES) dbc.setMap(role, sig)
            pump()
        }
    }

    companion object {
        const val SAMPLE_DBC = "zitto_demo_vehicle.dbc"
        const val APP_VERSION = "1.0.0"
    }

    // ================================================================ helpers
    private fun now() = System.currentTimeMillis() / 1000.0

    fun toast(msg: String, err: Boolean = false) {
        _toasts.tryEmit(Toast(msg, err))
    }

    /** Run [block] on the hub thread; errors become an error toast, success shows [ok]. */
    fun act(ok: String? = null, block: suspend Hub.() -> Unit) {
        scope.launch {
            try {
                block()
                if (ok != null) toast(ok)
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                toast(e.message ?: e.toString(), true)
            }
            dirty = true
        }
    }

    // ================================================================ link
    fun startScan(timeoutS: Int, nameFilter: String, onlyBridge: Boolean) {
        if (_scan.value.scanning) return
        _scan.value = ScanState(scanning = true, devices = _scan.value.devices)
        scope.launch {
            try {
                val list = BleScanner.scan(ctx, timeoutS, nameFilter, onlyBridge) { devs ->
                    _scan.value = ScanState(true, devs + BleScanner.simDevice())
                }
                _scan.value = ScanState(false, list + BleScanner.simDevice())
            } catch (e: CancellationException) {
                throw e
            } catch (e: Exception) {
                toast(e.message ?: "Scan failed", true)
                _scan.value = ScanState(false, listOf(BleScanner.simDevice()), e.message)
            }
        }
    }

    private fun attach(l: Link) {
        link?.let { it.onLine = {}; it.onState = {} }
        link = l
        l.onLine = { line -> scope.launch { ingest(line) } }
        l.onState = { dirty = true }
    }

    suspend fun connect(address: String, name: String?, autoReconnect: Boolean) {
        link?.let { if (it.connected || it.connecting) it.disconnect() }
        val l: Link = if (address == SIM_ADDRESS) SimLink(scope, sampleDb)
        else BleLink(ctx, scope).also { it.autoReconnect = autoReconnect }
        attach(l)
        try {
            l.connect(address, name)
        } catch (e: CancellationException) {
            throw e
        } catch (e: Exception) {
            throw HubError("Could not connect to ${name ?: address}: ${e.message}")
        }
        if (address != SIM_ADDRESS) {
            settings.lastAddress = address
            settings.lastName = name
            settings.save()
        }
        dirty = true
    }

    suspend fun disconnect() {
        link?.disconnect(true)
        dirty = true
    }

    // ================================================================ ingest
    private fun ingest(line: String) {
        val p = Parser.parse(line)
        val f = p.fields
        val t = p.t
        var dec: DbcDecode? = null
        when (p.type) {
            "IMU" -> {
                val ax = f.dbl("ax_mg"); val ay = f.dbl("ay_mg"); val az = f.dbl("az_mg")
                f["roll_deg"] = round2(Math.toDegrees(atan2(ay, if (az == 0.0) 1e-9 else az)))
                val h = hypot(ay, az)
                f["pitch_deg"] = round2(Math.toDegrees(atan2(-ax, if (h == 0.0) 1e-9 else h)))
                f["accel_g"] = round(sqrt(ax * ax + ay * ay + az * az)) / 1000.0
                latest = latest.copy(imu = f, imuT = t)
                rings["ax"]!!.add(t, ax); rings["ay"]!!.add(t, ay); rings["az"]!!.add(t, az)
                rings["gx"]!!.add(t, f.dbl("gx_mdps") / 1000); rings["gy"]!!.add(t, f.dbl("gy_mdps") / 1000)
                rings["gz"]!!.add(t, f.dbl("gz_mdps") / 1000)
            }
            "CSA" -> {
                latest = latest.copy(csa = f, csaT = t)
                rings["cur"]!!.add(t, f.dbl("current_ma")); rings["vol"]!!.add(t, f.dbl("voltage_mv"))
                rings["pow"]!!.add(t, f.dbl("power_mw"))
            }
            "STATUS" -> latest = latest.copy(status = f, statusT = t)
            "HEARTBEAT" -> latest = latest.copy(heartbeat = f)
            "CAN_STATUS" -> latest = if (f.long("bus") == 1L) latest.copy(can1 = f) else latest.copy(can2 = f)
            "FLM" -> latest = latest.copy(flm = f)
            "GPIO_STATUS" -> {
                @Suppress("UNCHECKED_CAST")
                val pins = (f["pins"] as? List<GpioPin>).orEmpty()
                latest = latest.copy(gpio = latest.gpio + pins.associateBy { it.id })
            }
            "BRIDGE_STATUS", "STATS" -> latest = latest.copy(bridge = (latest.bridge ?: emptyMap()) + f, bridgeT = t)
            "INFO" -> latest = latest.copy(info = f["text"] as? String)
            "PONG" -> pingT?.let {
                latest = latest.copy(pingMs = round((t - it) * 10000) / 10.0)
                pingT = null
            }
            "CMD_ACK" -> {
                latest = latest.copy(lastAck = f)
                Regex("""ESP gpio=(\d+) state=(\d)""").find(p.raw)?.let {
                    latest = latest.copy(espGpio = latest.espGpio + (it.groupValues[1].toInt() to it.groupValues[2].toInt()))
                }
            }
            "CAN" -> {
                @Suppress("UNCHECKED_CAST")
                val data = (f["data"] as? List<Int>).orEmpty()
                dec = dbc.decode(f.long("bus").toInt(), f.long("id"), f["ext"] == true, data, t)
            }
        }
        val rec = Rec(nextId++, t, p.raw, p.seq, p.type, p.tags, f, dec)
        store(rec)

        val iter = waiters.iterator()
        while (iter.hasNext()) {
            val (re, d) = iter.next()
            if (re.containsMatchIn(line) && !d.isCompleted) {
                d.complete(line)
                iter.remove()
            }
        }
    }

    private fun round2(v: Double) = round(v * 100) / 100

    private fun store(rec: Rec) {
        records.addLast(rec)
        if (records.size > maxRecs) records.removeFirst()
        recsVersion++
        counts[rec.type] = (counts[rec.type] ?: 0) + 1
        for (tg in rec.tags) counts["#$tg"] = (counts["#$tg"] ?: 0) + 1
        rateWindow.addLast(rec.t)
        while (rateWindow.size > 4000) rateWindow.removeFirst()
        if (recWriter != null) record(rec)

        // ribbon lanes: CAN1, CAN2, IMU, CSA, SYS
        val lane = when {
            "CAN1" in rec.tags -> 0
            "CAN2" in rec.tags -> 1
            "IMU" in rec.tags -> 2
            "CSA" in rec.tags -> 3
            else -> 4
        }
        ribbon.addLast(lane to rec.t)
        while (ribbon.size > 4000 || (ribbon.isNotEmpty() && rec.t - ribbon.first().second > 12)) ribbon.removeFirst()

        feedConsole(rec)

        val d = rec.dbc
        if (rec.type == "CAN" && d != null && plotSel.isNotEmpty()) {
            val bus = rec.fields.long("bus")
            for ((name, v) in d.signals) {
                val key = "$bus:${d.message}.$name"
                if (key in plotSel && v.v != null) plotRings.getOrPut(key) { Ring(6000) }.add(rec.t, v.v)
            }
        }
        dirty = true
    }

    private val conTags = setOf("ESP32", "BRIDGE", "CMD", "TX", "RAW", "OTHER", "OTA")
    private val failRe = Regex("fail|ERR")

    private fun feedConsole(r: Rec) {
        val isCon = r.tags.any { it in conTags } || (r.type == "LOG" && "CAN1" !in r.tags && "CAN2" !in r.tags)
        if (isCon) {
            val cls = if (r.type == "TX") "tx" else if ("ERR" in r.tags || failRe.containsMatchIn(r.raw)) "er" else "rx"
            console.addLast(ConLine(r.t, if (r.type == "TX") (r.fields["cmd"] as? String ?: r.raw) else r.raw, cls))
            while (console.size > 400) console.removeFirst()
        }
        val flashLog = (r.type == "LOG" && "FLASH" in r.tags) || (r.type == "LOG" && flashWait > 0 && r.t - flashWait < 3)
        if (flashLog || r.type == "FLASH_DATA") {
            val text = if (r.type == "FLASH_DATA") "FLASH_DATA ${r.summary}" else (r.fields["text"] as? String ?: r.raw)
            flashOut.addLast(ConLine(r.t, text, if (failRe.containsMatchIn(text)) "er" else "rx"))
            while (flashOut.size > 200) flashOut.removeFirst()
            if ("FLASH" !in r.tags) flashWait = 0.0
        }
    }

    private fun addTx(cmd: String) {
        store(Rec(nextId++, now(), cmd, null, "TX", listOf("TX"), mapOf("cmd" to cmd)))
    }

    private fun txFailed(cmd: String, why: String) {
        store(Rec(nextId++, now(), "TX FAILED $cmd: $why", null, "TX_ERR", listOf("TX", "ERR"), mapOf("cmd" to cmd, "error" to why)))
    }

    private fun rate(): Double {
        val n = now()
        while (rateWindow.isNotEmpty() && n - rateWindow.first() > 5.0) rateWindow.removeFirst()
        return round(rateWindow.size / 5.0 * 10) / 10
    }

    private fun expect(pattern: String): CompletableDeferred<String> {
        val d = CompletableDeferred<String>()
        waiters += Regex(pattern) to d
        return d
    }

    // ================================================================ commands
    suspend fun send(cmd: String) {
        val l = link
        if (l == null || !l.connected) throw HubError("Not connected to a bridge. Connect first.")
        if (cmd.uppercase() == "PING") pingT = now()
        addTx(cmd)
        try {
            l.write(cmd)
        } catch (e: CancellationException) {
            throw e
        } catch (e: IllegalArgumentException) {
            txFailed(cmd, e.message ?: "invalid")
            throw HubError(e.message ?: "invalid command")
        } catch (e: Exception) {
            txFailed(cmd, e.message ?: "error")
            throw HubError("BLE write failed: ${e.message}")
        }
    }

    fun sendCmd(cmd: String, ok: String? = null) {
        val c = cmd.trim()
        if (c.isEmpty()) { toast("Type a command first.", true); return }
        act(ok) { send(c) }
    }

    fun module(name: String, on: Boolean) = act("$name ${if (on) "enable" else "disable"} sent") { send(Protocol.moduleEn(name, on)) }
    fun statusReq() = act("Status requested") { send(Protocol.statusReq()) }
    fun mcuReset() = act("Reset command sent") { send(Protocol.mcuReset()) }
    fun led(period: Int, duty: Int) = act("LED settings sent") { send(Protocol.ledCtrl(period, duty)) }

    fun flashRead() = act { flashWait = now(); send(Protocol.flashRead()) }
    fun flashDelete() = act("Delete sent") { send(Protocol.flashDelete()) }
    fun flashWrite(text: String, isHex: Boolean) = act("Write sent") {
        val data = if (isHex) {
            try { Protocol.parseHex(text) } catch (_: Exception) { throw HubError("Hex data is not valid. Use pairs like 01 A2 FF.") }
        } else text.toByteArray()
        if (data.isEmpty()) throw HubError("Enter text or hex to write.")
        val maxLen = maxOf(8, ((link?.maxWrite ?: 106) - 6) / 2)
        if (data.size > maxLen) throw HubError("Record too long for one BLE write. Keep it under $maxLen bytes.")
        send(Protocol.flashWrite(data))
    }

    fun s32Gpio(id: Int, dir: Int, state: Int, raw: Boolean) = act("GPIO $id command sent") {
        if (id !in Protocol.S32_GPIO_MAP) throw HubError("S32K GPIO ID must be 1..13")
        send(if (raw) Protocol.s32GpioRaw(id, dir, state) else Protocol.s32Gpio(id, dir, state))
    }

    fun espGpio(pin: Int, state: Int) = act {
        if (pin !in Protocol.ESP_ALLOWED_PINS) throw HubError("GPIO$pin is not in the bridge's allowed list")
        send(Protocol.espGpio(pin, state))
    }

    // ================================================================ DBC
    fun loadDbc(nameIn: String, text: String, buses: List<Int>) = act {
        val name = File(nameIn).name
        if (!name.lowercase().endsWith(".dbc")) throw HubError("Choose a .dbc file ($name).")
        val db = try { dbc.load(name, text, buses) } catch (e: Exception) { throw HubError("DBC parse error in $name: ${e.message}") }
        File(dbcDir, name).writeText(text)
        settings.dbc[name] = buses
        settings.save()
        toast("$name: ${db.messages.size} messages loaded")
    }

    fun loadSampleDbc() = act("Demo DBC loaded on CAN1 and CAN2") {
        dbc.load(SAMPLE_DBC, sampleText, listOf(1, 2))
        File(dbcDir, SAMPLE_DBC).writeText(sampleText)
        settings.dbc[SAMPLE_DBC] = listOf(1, 2)
        settings.save()
    }

    fun removeDbc(name: String) = act("DBC removed") {
        dbc.remove(name)
        settings.dbc.remove(name)
        File(dbcDir, name).delete()
        settings.save()
    }

    fun setDbcBuses(name: String, buses: List<Int>) = act("Bus assignment saved") {
        dbc.setBuses(name, buses)
        if (name in settings.dbc) { settings.dbc[name] = buses; settings.save() }
    }

    fun setVehicleMap(role: String, sig: String?) = act("Mapping saved") {
        dbc.setMap(role, sig)
        settings.vehicleMap[role] = sig
        settings.save()
    }

    fun setPlotSel(sel: List<String>) = act {
        plotSel = sel.take(6)
        plotRings.keys.retainAll(plotSel.toSet())
        ctx.getSharedPreferences("vcum", Context.MODE_PRIVATE).edit().putString("plotSel", plotSel.joinToString("\n")).apply()
    }

    fun resetDbcStats() = act { dbc.resetStats() }

    // ================================================================ filters / clear / record
    fun addFilter(f: CustomFilter) = act("Filter \"${f.name}\" added and active") {
        if (f.regex) try { Regex(f.expr) } catch (e: Exception) { throw HubError("That regex is not valid: ${e.message}") }
        if (settings.filters.any { it.name == f.name }) throw HubError("A filter with that name exists. Pick another name.")
        settings.filters += f
        settings.save()
    }

    fun removeFilter(name: String) = act {
        settings.filters.removeAll { it.name == name }
        settings.save()
    }

    fun clear() = act {
        records.clear()
        recsVersion++
        counts.clear()
        rings.values.forEach { it.clear() }
        plotRings.values.forEach { it.clear() }
        ribbon.clear()
        dbc.resetStats()
    }

    fun toggleRecord() = act {
        if (recWriter != null) {
            val n = stopRecord()
            toast("Recording saved: $n")
        } else {
            val n = startRecord()
            toast("Recording to $n")
        }
    }

    private fun startRecord(): String {
        val name = "session_${Fmt.fileStamp()}.csv"
        logDir.mkdirs()
        val w = File(logDir, name).bufferedWriter()
        w.write("pc_time,seq,type,tags,raw,decoded\n")
        recWriter = w
        recName = name
        recRows = 0
        recSince = now()
        return name
    }

    private fun stopRecord(): String? {
        val w = recWriter ?: return null
        try { w.close() } catch (_: Exception) { }
        recWriter = null
        return recName.also { recName = null }
    }

    private fun record(r: Rec) {
        val w = recWriter ?: return
        try {
            w.write(listOf(Fmt.stamp(r.t), r.seq?.toString() ?: "", r.type, r.tags.joinToString("|"), r.raw, r.decodeText)
                .joinToString(",") { Fmt.csv(it) })
            w.write("\n")
            recRows++
            if (recRows % 50 == 0L) w.flush()
        } catch (e: Exception) {
            stopRecord()
            toast("Recording stopped: ${e.message}", true)
        }
    }

    fun logFiles(): List<File> = (logDir.listFiles { f -> f.name.endsWith(".csv") } ?: emptyArray()).sortedByDescending { it.name }

    /** Write an export into the cache for sharing. */
    suspend fun writeExport(name: String, content: String): File = withContext(kotlinx.coroutines.Dispatchers.IO) {
        exportDir.mkdirs()
        File(exportDir, name).also { it.writeText(content) }
    }

    // ================================================================ OTA
    fun otaLoad(nameIn: String, data: ByteArray) = act {
        if (otaState in RUNNING_STATES) throw HubError("Transfer in progress. Abort it first.")
        if (data.isEmpty()) throw HubError("The image is empty.")
        val name = File(nameIn).name
        otaImage = data
        otaName = name
        otaCrc = Protocol.crc32(data)
        otaState = "LOADED"; otaSent = 0; otaError = null; otaStarted = null; otaFinished = null
        try { File(otaDir, name).writeBytes(data) } catch (_: Exception) { }
        otaEv("Loaded $name: ${data.size} bytes, CRC32 " + "0x%08X".format(otaCrc))
        toast("$name loaded")
    }

    private fun otaEv(s: String) {
        otaEvents.addLast("${Fmt.hms()} $s")
        while (otaEvents.size > 60) otaEvents.removeFirst()
    }

    private fun otaInfo(): OtaInfo {
        val size = otaImage?.size ?: 0
        val st = otaStarted
        val el = if (st != null) (otaFinished ?: now()) - st else 0.0
        val rate = if (el > 0) otaSent / el else 0.0
        return OtaInfo(
            state = otaState, name = otaName, size = size, crc = otaCrc?.let { "0x%08X".format(it) },
            sent = otaSent, pct = if (size > 0) round(1000.0 * otaSent / size) / 10 else 0.0,
            rateBps = rate.toLong(), etaS = if (rate > 0) ((size - otaSent) / rate).toLong() else null,
            elapsedS = round(el * 10) / 10, chunk = otaChunk, delayMs = otaDelay, error = otaError,
            events = otaEvents.toList(),
        )
    }

    fun maxOtaChunk(): Int = maxOf(8, ((link?.maxWrite ?: 244) - 6) / 2)

    fun otaStart(chunk: Int, delayMs: Int) = act("Update started") {
        val img = otaImage ?: throw HubError("Load a firmware image first.")
        if (otaState in RUNNING_STATES) throw HubError("Transfer already running.")
        val l = link
        if (l == null || !l.connected) throw HubError("Connect to the bridge before starting OTA.")
        otaChunk = chunk.coerceIn(8, minOf(maxOtaChunk(), 240))
        otaDelay = maxOf(0, delayMs)
        otaState = "STARTING"
        otaJob = scope.launch { otaRun(img) }
    }

    private suspend fun otaRun(img: ByteArray) {
        otaState = "STARTING"; otaSent = 0; otaError = null
        otaStarted = now(); otaFinished = null
        val crc = otaCrc ?: Protocol.crc32(img)
        val fail = expect("OTA:(write_fail|bad_len)")
        try {
            otaEv("START size=${img.size} crc=" + "0x%08X".format(crc) + " chunk=${otaChunk}B delay=${otaDelay}ms")
            val fut = expect("OTA:(start_ok|start_fail|bad_len)")
            send(Protocol.otaStart(img.size, crc))
            val res = withTimeout(6000) { fut.await() }
            if ("start_ok" !in res) throw HubError("MCU rejected OTA start (${res.trim().split(" ").last()})")
            otaState = "SENDING"
            var off = 0
            while (off < img.size) {
                if (fail.isCompleted) throw HubError("MCU reported OTA:write_fail")
                val part = img.copyOfRange(off, minOf(img.size, off + otaChunk))
                send(Protocol.otaData(part))
                off += part.size
                otaSent = off
                dirty = true
                if (otaDelay > 0) delay(otaDelay.toLong())
            }
            otaState = "FINISHING"
            delay(200)
            if (fail.isCompleted) throw HubError("MCU reported OTA:write_fail")
            val fin = expect("OTA:(ok|verify_fail)")
            send(Protocol.otaFinish())
            val r2 = withTimeout(15000) { fin.await() }
            if ("verify_fail" in r2) throw HubError("MCU CRC verification failed (OTA:verify_fail)")
            otaState = "COMPLETE"
            otaEv("MCU verified image (OTA:ok). Reset the MCU to apply if your bootloader expects it.")
            toast("Firmware update complete")
        } catch (e: TimeoutCancellationException) {
            otaState = "FAILED"; otaError = "No response from MCU (timeout)"
            otaEv(otaError!!)
            withContext(NonCancellable) { try { send(Protocol.otaAbort()) } catch (_: Exception) { } }
        } catch (e: CancellationException) {
            otaState = "ABORTED"
            otaEv("Transfer cancelled by user")
            withContext(NonCancellable) { try { send(Protocol.otaAbort()) } catch (_: Exception) { } }
        } catch (e: Exception) {
            otaState = "FAILED"; otaError = e.message
            otaEv("FAILED: ${e.message}")
            toast("OTA failed: ${e.message}", true)
            withContext(NonCancellable) { try { send(Protocol.otaAbort()) } catch (_: Exception) { } }
        } finally {
            otaFinished = now()
            if (!fail.isCompleted) fail.cancel()
            waiters.removeAll { it.second.isCompleted }
            dirty = true
        }
    }

    fun otaAbort() = act("Abort sent") {
        val j = otaJob
        if (j != null && j.isActive) {
            j.cancel()
        } else {
            try { send(Protocol.otaAbort()); otaEv("ABORT sent") } catch (_: HubError) { }
            otaState = if (otaImage != null) "ABORTED" else "IDLE"
        }
    }

    // ================================================================ publish
    private suspend fun pump() {
        var k = 0
        while (true) {
            delay(100)
            k++
            if (dirty || k % 5 == 0) {
                dirty = false
                publish()
            }
        }
    }

    private fun publish() {
        if (recsSnapVersion != recsVersion) {
            recsSnap = ArrayList(records)
            recsSnapVersion = recsVersion
        }
        val n = now()
        val l = link
        val cur10 = rings["cur"]!!.since(n - 10)
        _state.value = HubState(
            link = l?.info() ?: LinkInfo(),
            rate = rate(),
            counts = HashMap(counts),
            latest = latest,
            vehicle = dbc.vehicleSnapshot(n),
            ota = otaInfo(),
            recording = recName?.let { RecInfo(it, recRows, recSince) },
            dbcList = dbc.list(),
            dbcs = dbc.dbcs.mapValues { it.value.db },
            signals = dbc.signals.values.toList(),
            messages = dbc.messages.values.toList(),
            records = recsSnap,
            recsVersion = recsVersion,
            console = console.toList(),
            flashOut = flashOut.toList(),
            charts = rings.mapValues { it.value.snap() },
            plot = plotRings.mapValues { it.value.snap() },
            plotSel = plotSel,
            ribbon = ribbon.toList(),
            filters = settings.filters.toList(),
            vehicleMap = LinkedHashMap(dbc.vehicleMap),
            allSignals = dbc.allSignalNames().sorted(),
            services = if (l != null && l.connected) l.services() else emptyList(),
            lastDevice = settings.lastAddress?.let { it to settings.lastName },
            csaAvg10 = if (cur10.isEmpty()) null else cur10.average(),
            csaPeak10 = cur10.maxOrNull(),
        )
    }
}
