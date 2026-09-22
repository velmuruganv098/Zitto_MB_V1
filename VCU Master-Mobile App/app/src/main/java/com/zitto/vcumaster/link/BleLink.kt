package com.zitto.vcumaster.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.BluetoothStatusCodes
import android.content.Context
import android.os.Build
import kotlinx.coroutines.CancellationException
import kotlinx.coroutines.CompletableDeferred
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.TimeoutCancellationException
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlinx.coroutines.sync.Mutex
import kotlinx.coroutines.sync.withLock
import kotlinx.coroutines.withContext
import kotlinx.coroutines.withTimeout

/**
 * Real hardware over BLE: Nordic UART service on the ESP32-S3 bridge.
 * connect -> MTU 247 -> discover -> enable TX notify -> INFO + STATS hello.
 */
@SuppressLint("MissingPermission")
class BleLink(private val ctx: Context, private val scope: CoroutineScope) : Link() {
    override val kind = "ble"

    @Volatile var autoReconnect = true
    @Volatile private var userDisconnect = false
    @Volatile private var gatt: BluetoothGatt? = null
    @Volatile private var rxChar: BluetoothGattCharacteristic? = null
    @Volatile private var setupDone: CompletableDeferred<Unit>? = null
    @Volatile private var writeDone: CompletableDeferred<Int>? = null
    private val wlock = Mutex()
    private var reconnectJob: Job? = null

    private val adapter get() = ctx.getSystemService(BluetoothManager::class.java)?.adapter

    override suspend fun connect(address: String, name: String?) {
        closeGatt()
        userDisconnect = false
        error = null
        this.address = address
        this.name = name
        connecting = true
        connected = false
        emitState()

        val ad = adapter
        if (ad == null || !ad.isEnabled) {
            connecting = false
            error = "Bluetooth is off"
            emitState()
            throw LinkException("Bluetooth is off. Switch it on first.")
        }
        val dev = ad.getRemoteDevice(address)
        val done = CompletableDeferred<Unit>()
        setupDone = done
        withContext(Dispatchers.Main) {
            gatt = dev.connectGatt(ctx, false, callback, BluetoothDevice.TRANSPORT_LE)
        }
        try {
            withTimeout(20_000) { done.await() }
        } catch (e: TimeoutCancellationException) {
            failSetup("connect timed out")
        } catch (e: LinkException) {
            failSetup(e.message ?: "connect failed")
        }
        connecting = false
        connected = true
        connectedAt = now()
        emitState()
        scope.launch {
            delay(300)
            for (c in listOf("INFO", "STATS")) {
                try { write(c) } catch (e: CancellationException) { throw e } catch (_: Exception) { }
            }
        }
    }

    private fun failSetup(msg: String): Nothing {
        error = msg
        connecting = false
        connected = false
        closeGatt()
        emitState()
        throw LinkException(msg)
    }

    private fun closeGatt() {
        val g = gatt
        gatt = null
        rxChar = null
        if (g != null) {
            try { g.disconnect() } catch (_: Exception) { }
            try { g.close() } catch (_: Exception) { }
        }
        writeDone?.complete(-1)
    }

    private fun startReconnect() {
        reconnectJob?.cancel()
        val addr = address ?: return
        reconnectJob = scope.launch {
            var d = 1000.0
            while (!userDisconnect && !connected) {
                delay(d.toLong())
                if (userDisconnect) break
                reconnects++
                try {
                    connect(addr, name)
                    return@launch
                } catch (e: CancellationException) {
                    throw e
                } catch (e: Exception) {
                    error = "reconnect: ${e.message}"
                    emitState()
                    d = minOf(d * 1.7, 15000.0)
                }
            }
        }
    }

    override suspend fun disconnect(user: Boolean) {
        if (user) {
            userDisconnect = true
            reconnectJob?.cancel()
        }
        closeGatt()
        connected = false
        connecting = false
        emitState()
    }

    override suspend fun write(cmd: String) {
        if (!connected) throw LinkException("BLE not connected")
        val data = cmd.toByteArray(Charsets.US_ASCII)
        if (data.size > maxWrite) throw IllegalArgumentException("command ${data.size} B exceeds ATT payload $maxWrite B")
        wlock.withLock {
            val g = gatt
            val c = rxChar
            if (g == null || c == null) throw LinkException("BLE not connected")
            val done = CompletableDeferred<Int>()
            writeDone = done
            val started = if (Build.VERSION.SDK_INT >= 33) {
                g.writeCharacteristic(c, data, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) == BluetoothStatusCodes.SUCCESS
            } else {
                @Suppress("DEPRECATION")
                run {
                    c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                    c.value = data
                    g.writeCharacteristic(c)
                }
            }
            if (!started) {
                txErrors++
                throw LinkException("GATT write rejected (busy)")
            }
            val status = try {
                withTimeout(5000) { done.await() }
            } catch (e: TimeoutCancellationException) {
                txErrors++
                throw LinkException("GATT write timed out")
            }
            if (status != BluetoothGatt.GATT_SUCCESS) {
                txErrors++
                throw LinkException("GATT write failed (status $status)")
            }
            txCmds++
        }
    }

    override fun services(): List<GattService> {
        val g = gatt ?: return emptyList()
        return g.services.map { s ->
            GattService(
                s.uuid.toString(),
                if (s.uuid == NUS_SERVICE) "Nordic UART" else describe(s.uuid.toString()),
                s.characteristics.map { c ->
                    GattChar(c.uuid.toString(), when (c.uuid) {
                        NUS_RX -> "RX (write)"
                        NUS_TX -> "TX (notify)"
                        else -> ""
                    }, props(c.properties))
                },
            )
        }
    }

    private fun describe(u: String) = when (u.substring(4, 8)) {
        "1800" -> "Generic Access"
        "1801" -> "Generic Attribute"
        "180a" -> "Device Information"
        else -> ""
    }

    private fun props(p: Int): List<String> = buildList {
        if (p and BluetoothGattCharacteristic.PROPERTY_READ != 0) add("read")
        if (p and BluetoothGattCharacteristic.PROPERTY_WRITE != 0) add("write")
        if (p and BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE != 0) add("write-without-response")
        if (p and BluetoothGattCharacteristic.PROPERTY_NOTIFY != 0) add("notify")
        if (p and BluetoothGattCharacteristic.PROPERTY_INDICATE != 0) add("indicate")
    }

    private fun handleNotify(value: ByteArray) {
        // The bridge publishes exactly one line per notification with no newline.
        val text = String(value, Charsets.UTF_8).replace("\r", "")
        for (p in text.split("\n")) if (p.isNotBlank()) emitLine(p)
    }

    /** A callback from a GATT object we already replaced. */
    private fun stale(g: BluetoothGatt) = gatt.let { it != null && it !== g }

    private val callback = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (stale(g)) { g.close(); return }
            if (newState == BluetoothProfile.STATE_CONNECTED && status == BluetoothGatt.GATT_SUCCESS) {
                g.requestConnectionPriority(BluetoothGatt.CONNECTION_PRIORITY_HIGH)
                if (!g.requestMtu(247)) g.discoverServices()
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED || status != BluetoothGatt.GATT_SUCCESS) {
                val was = connected
                connected = false
                setupDone?.let { if (!it.isCompleted) it.completeExceptionally(LinkException("GATT error $status")) }
                writeDone?.complete(-1)
                try { g.close() } catch (_: Exception) { }
                gatt = null
                rxChar = null
                if (was) error = "link lost (status $status)"
                emitState()
                if (was && autoReconnect && !userDisconnect) startReconnect()
            }
        }

        override fun onMtuChanged(g: BluetoothGatt, mtu: Int, status: Int) {
            if (stale(g)) return
            this@BleLink.mtu = if (status == BluetoothGatt.GATT_SUCCESS) mtu else 23
            g.discoverServices()
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            if (stale(g)) return
            val svc = g.getService(NUS_SERVICE)
            val tx = svc?.getCharacteristic(NUS_TX)
            val rx = svc?.getCharacteristic(NUS_RX)
            if (status != BluetoothGatt.GATT_SUCCESS || tx == null || rx == null) {
                setupDone?.completeExceptionally(LinkException("Nordic UART service not found on this device"))
                return
            }
            rxChar = rx
            g.setCharacteristicNotification(tx, true)
            val d = tx.getDescriptor(CCCD)
            if (d == null) {
                setupDone?.complete(Unit)
                return
            }
            val ok = if (Build.VERSION.SDK_INT >= 33) {
                g.writeDescriptor(d, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE) == BluetoothStatusCodes.SUCCESS
            } else {
                @Suppress("DEPRECATION")
                run {
                    d.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                    g.writeDescriptor(d)
                }
            }
            if (!ok) setupDone?.completeExceptionally(LinkException("Could not enable notifications"))
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            if (stale(g)) return
            if (status == BluetoothGatt.GATT_SUCCESS) setupDone?.complete(Unit)
            else setupDone?.completeExceptionally(LinkException("Enable notify failed (status $status)"))
        }

        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            writeDone?.complete(status)
        }

        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            if (c.uuid == NUS_TX) handleNotify(value)
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            if (Build.VERSION.SDK_INT < 33 && c.uuid == NUS_TX) {
                @Suppress("DEPRECATION")
                c.value?.let { handleNotify(it) }
            }
        }
    }
}
