package com.zitto.vcumaster.core

import java.text.SimpleDateFormat
import java.util.Date
import java.util.Locale
import kotlin.math.abs
import kotlin.math.floor

/** Formatting helpers shared by all screens (same rules as the desktop UI). */
object Fmt {
    fun nowS() = System.currentTimeMillis() / 1000.0

    fun num(v: Double?): String {
        if (v == null || v.isNaN()) return "–"
        if (v == floor(v) && abs(v) < 1e12) return v.toLong().toString()
        val a = abs(v)
        return when {
            a >= 100 -> "%.1f".format(Locale.US, v)
            a >= 1 -> "%.2f".format(Locale.US, v)
            else -> "%.3f".format(Locale.US, v)
        }
    }

    fun fix(v: Double?, digits: Int): String = if (v == null) "–" else "%.${digits}f".format(Locale.US, v)

    fun dur(ms: Double?): String {
        if (ms == null) return "–"
        var s = floor(ms / 1000).toLong()
        val d = s / 86400; s %= 86400
        val h = s / 3600; s %= 3600
        val m = s / 60; s %= 60
        return (if (d > 0) "${d}d " else "") + (if (h > 0 || d > 0) "${h}h " else "") + "${m}m ${s}s"
    }

    private val clockFmt = ThreadLocal.withInitial { SimpleDateFormat("HH:mm:ss.SSS", Locale.US) }
    private val stampFmt = ThreadLocal.withInitial { SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US) }
    private val fileFmt = ThreadLocal.withInitial { SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US) }
    private val hmsFmt = ThreadLocal.withInitial { SimpleDateFormat("HH:mm:ss", Locale.US) }

    fun clock(t: Double): String = clockFmt.get()!!.format(Date((t * 1000).toLong()))
    fun stamp(t: Double): String = stampFmt.get()!!.format(Date((t * 1000).toLong()))
    fun fileStamp(): String = fileFmt.get()!!.format(Date())
    fun hms(): String = hmsFmt.get()!!.format(Date())

    fun ago(t: Double?): String {
        if (t == null || t == 0.0) return "–"
        val s = nowS() - t
        return when {
            s < 1 -> "now"
            s < 60 -> "${s.toInt()} s"
            else -> "${(s / 60).toInt()} min"
        }
    }

    fun hex2(n: Int) = "%02X".format(n)
    fun hexId(id: Long, ext: Boolean) = "0x" + id.toString(16).uppercase().padStart(if (ext) 8 else 3, '0')

    fun csv(s: String): String =
        if (s.any { it == ',' || it == '"' || it == '\n' || it == '\r' }) "\"" + s.replace("\"", "\"\"") + "\"" else s

    fun thousands(n: Long): String = "%,d".format(Locale.US, n)
}
