package com.zitto.vcumaster.ui

import android.content.Context
import androidx.compose.runtime.getValue
import androidx.compose.runtime.mutableStateOf
import androidx.compose.runtime.setValue

/** Per-device UI conveniences (the desktop app's localStorage). Main thread only. */
class UiPrefs(ctx: Context) {
    private val sp = ctx.getSharedPreferences("vcum_ui", Context.MODE_PRIVATE)

    private fun list(k: String) = sp.getString(k, "")!!.split("\n").filter { it.isNotEmpty() }
    private fun putList(k: String, v: Collection<String>) = sp.edit().putString(k, v.joinToString("\n")).apply()

    var theme by mutableStateOf(sp.getString("theme", "system")!!)
        private set
    fun setThemeMode(m: String) { theme = m; sp.edit().putString("theme", m).apply() }

    var tab by mutableStateOf(sp.getString("tab", "connect")!!)
        private set
    fun setTabName(t: String) { tab = t; sp.edit().putString("tab", t).apply() }

    var tagSel by mutableStateOf(list("tagSel").toSet())
        private set
    fun setTags(s: Set<String>) { tagSel = s; putList("tagSel", s) }

    var cfOn by mutableStateOf(list("cfOn").toSet())
        private set
    fun updateCfOn(s: Set<String>) { cfOn = s; putList("cfOn", s) }

    var cmdHist by mutableStateOf(list("cmdHist"))
        private set
    fun pushHist(c: String) {
        if (cmdHist.lastOrNull() == c) return
        cmdHist = (cmdHist.filter { it != c } + c).takeLast(50)
        putList("cmdHist", cmdHist)
    }

    var scanTime by mutableStateOf(sp.getInt("scanTime", 5))
        private set
    fun setScanSeconds(v: Int) { scanTime = v; sp.edit().putInt("scanTime", v).apply() }

    var nameFilter by mutableStateOf(sp.getString("nameFilter", "")!!)
        private set
    fun setName(v: String) { nameFilter = v; sp.edit().putString("nameFilter", v).apply() }

    var onlyBridge by mutableStateOf(sp.getBoolean("onlyBridge", false))
        private set
    fun setOnly(v: Boolean) { onlyBridge = v; sp.edit().putBoolean("onlyBridge", v).apply() }

    var autoRe by mutableStateOf(sp.getBoolean("autoRe", true))
        private set
    fun setAuto(v: Boolean) { autoRe = v; sp.edit().putBoolean("autoRe", v).apply() }

    var keepAwake by mutableStateOf(sp.getBoolean("keepAwake", true))
        private set
    fun setAwake(v: Boolean) { keepAwake = v; sp.edit().putBoolean("keepAwake", v).apply() }

    var otaChunk by mutableStateOf(sp.getInt("otaChunk", 96))
        private set
    var otaDelay by mutableStateOf(sp.getInt("otaDelay", 30))
        private set
    fun setOta(chunk: Int, delay: Int) {
        otaChunk = chunk; otaDelay = delay
        sp.edit().putInt("otaChunk", chunk).putInt("otaDelay", delay).apply()
    }

    var gpioRaw by mutableStateOf(sp.getBoolean("gpioRaw", true))
        private set
    fun setGpioVia(raw: Boolean) { gpioRaw = raw; sp.edit().putBoolean("gpioRaw", raw).apply() }

    var winS by mutableStateOf(sp.getInt("winS", 30))
        private set
    fun setWindow(s: Int) { winS = s; sp.edit().putInt("winS", s).apply() }
}
