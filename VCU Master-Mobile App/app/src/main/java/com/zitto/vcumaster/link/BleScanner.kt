package com.zitto.vcumaster.link

import android.annotation.SuppressLint
import android.bluetooth.BluetoothManager
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import kotlinx.coroutines.delay
import java.util.concurrent.ConcurrentHashMap

@SuppressLint("MissingPermission")
object BleScanner {

    fun simDevice() = ScanDev(SIM_ADDRESS, "Zitto_MB_V1_Bridge (simulator)", -42, null, listOf(NUS_SERVICE.toString()), emptyMap(), true)

    private fun failText(code: Int) = when (code) {
        ScanCallback.SCAN_FAILED_ALREADY_STARTED -> "a scan is already running"
        ScanCallback.SCAN_FAILED_APPLICATION_REGISTRATION_FAILED -> "the Bluetooth stack refused the scan, toggle Bluetooth and retry"
        ScanCallback.SCAN_FAILED_FEATURE_UNSUPPORTED -> "BLE scanning is not supported"
        6 -> "Android limits apps to 5 scans per 30 s, wait a moment and retry"
        else -> "error $code"
    }

    suspend fun scan(
        ctx: Context,
        timeoutS: Int,
        nameFilter: String,
        onlyBridge: Boolean,
        onUpdate: (List<ScanDev>) -> Unit,
    ): List<ScanDev> {
        val adapter = ctx.getSystemService(BluetoothManager::class.java)?.adapter
            ?: throw LinkException("This phone has no Bluetooth adapter")
        if (!adapter.isEnabled) throw LinkException("Bluetooth is off. Switch it on and scan again.")
        val scanner = adapter.bluetoothLeScanner ?: throw LinkException("Bluetooth LE scanner unavailable")

        val found = ConcurrentHashMap<String, ScanDev>()
        var failed: Int? = null

        fun add(r: ScanResult) {
            val rec = r.scanRecord
            val devName: String? = try { r.device.name } catch (_: SecurityException) { null }
            val name = rec?.deviceName ?: devName ?: ""
            val uuids = rec?.serviceUuids?.map { it.uuid.toString().lowercase() } ?: emptyList()
            val mfg = HashMap<Int, String>()
            rec?.manufacturerSpecificData?.let { sa ->
                for (i in 0 until sa.size()) mfg[sa.keyAt(i)] = sa.valueAt(i).joinToString("") { "%02x".format(it) }
            }
            val tx = rec?.txPowerLevel?.takeIf { it != Int.MIN_VALUE }
            found[r.device.address] = ScanDev(
                address = r.device.address,
                name = name.ifEmpty { "(unnamed)" },
                rssi = r.rssi,
                txPower = tx,
                uuids = uuids,
                manufacturer = mfg,
                bridge = NUS_SERVICE.toString() in uuids || name == DEFAULT_NAME,
            )
        }

        fun sorted(): List<ScanDev> = found.values
            .filter { nameFilter.isBlank() || it.name.contains(nameFilter, ignoreCase = true) }
            .filter { !onlyBridge || it.bridge }
            .sortedWith(compareBy<ScanDev>({ !it.bridge }, { -(it.rssi ?: -999) }))

        val cb = object : ScanCallback() {
            override fun onScanResult(callbackType: Int, result: ScanResult) = add(result)
            override fun onBatchScanResults(results: MutableList<ScanResult>) = results.forEach { add(it) }
            override fun onScanFailed(errorCode: Int) { failed = errorCode }
        }
        val settings = ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build()
        scanner.startScan(null, settings, cb)
        try {
            repeat(timeoutS * 4) {
                delay(250)
                failed?.let { throw LinkException("Scan failed: ${failText(it)}") }
                onUpdate(sorted())
            }
        } finally {
            try { scanner.stopScan(cb) } catch (_: Exception) { }
        }
        return sorted()
    }
}
