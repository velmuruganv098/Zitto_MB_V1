package com.zitto.vcumaster.link

import java.util.UUID

val NUS_SERVICE: UUID = UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e")
val NUS_RX: UUID = UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e")   // phone -> ESP32 (write)
val NUS_TX: UUID = UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e")   // ESP32 -> phone (notify)
val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
const val DEFAULT_NAME = "Zitto_MB_V1_Bridge"
const val SIM_ADDRESS = "SIM"

data class LinkInfo(
    val kind: String? = null,
    val connected: Boolean = false,
    val connecting: Boolean = false,
    val address: String? = null,
    val name: String? = null,
    val since: Double? = null,
    val rxLines: Long = 0,
    val rxBytes: Long = 0,
    val txCmds: Long = 0,
    val txErrors: Long = 0,
    val lastRx: Double? = null,
    val mtu: Int? = null,
    val error: String? = null,
    val reconnects: Int = 0,
)

data class GattChar(val uuid: String, val description: String, val properties: List<String>)
data class GattService(val uuid: String, val description: String, val chars: List<GattChar>)

data class ScanDev(
    val address: String,
    val name: String,
    val rssi: Int?,
    val txPower: Int?,
    val uuids: List<String>,
    val manufacturer: Map<Int, String>,
    val bridge: Boolean,
)

class LinkException(msg: String) : Exception(msg)

/** Common transport contract shared by the BLE link and the simulator. */
abstract class Link {
    abstract val kind: String

    var onLine: (String) -> Unit = {}
    var onState: () -> Unit = {}

    @Volatile var connected = false
    @Volatile var connecting = false
    @Volatile var address: String? = null
    @Volatile var name: String? = null
    @Volatile var connectedAt: Double? = null
    @Volatile var rxLines = 0L
    @Volatile var rxBytes = 0L
    @Volatile var txCmds = 0L
    @Volatile var txErrors = 0L
    @Volatile var lastRx: Double? = null
    @Volatile var mtu: Int? = null
    @Volatile var error: String? = null
    @Volatile var reconnects = 0

    protected fun now() = System.currentTimeMillis() / 1000.0

    protected fun emitLine(line: String) {
        rxLines++
        rxBytes += line.length
        lastRx = now()
        onLine(line)
    }

    protected fun emitState() = onState()

    fun info() = LinkInfo(
        kind, connected, connecting, address, name, connectedAt, rxLines, rxBytes, txCmds, txErrors,
        lastRx, mtu, error, reconnects,
    )

    /** Largest ATT write payload. */
    val maxWrite: Int get() = maxOf(20, (mtu ?: 23) - 3)

    abstract suspend fun connect(address: String, name: String?)
    abstract suspend fun disconnect(user: Boolean = true)
    abstract suspend fun write(cmd: String)
    abstract fun services(): List<GattService>
}
