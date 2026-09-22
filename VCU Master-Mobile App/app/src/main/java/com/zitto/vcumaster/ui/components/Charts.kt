package com.zitto.vcumaster.ui.components

import androidx.compose.animation.core.animateFloatAsState
import androidx.compose.animation.core.tween
import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.Box
import androidx.compose.foundation.layout.Column
import androidx.compose.foundation.layout.Row
import androidx.compose.foundation.layout.Spacer
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.height
import androidx.compose.foundation.layout.padding
import androidx.compose.foundation.layout.width
import androidx.compose.material3.Text
import androidx.compose.runtime.Composable
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.geometry.Size
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.StrokeCap
import androidx.compose.ui.graphics.StrokeJoin
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.graphics.drawscope.clipPath
import androidx.compose.ui.graphics.drawscope.rotate
import androidx.compose.ui.graphics.drawscope.translate
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.text.style.TextAlign
import androidx.compose.ui.text.style.TextOverflow
import androidx.compose.ui.unit.Dp
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.SeriesSnap
import com.zitto.vcumaster.core.VehEntry
import com.zitto.vcumaster.ui.theme.LocalVcu
import com.zitto.vcumaster.ui.theme.Mono
import java.util.Locale
import kotlin.math.abs
import kotlin.math.ceil
import kotlin.math.floor
import kotlin.math.log10
import kotlin.math.pow

data class ChartSeries(val label: String, val color: Color, val data: SeriesSnap)

private fun niceStep(range: Double, target: Int): Double {
    if (range <= 0) return 1.0
    val raw = range / target
    val mag = 10.0.pow(floor(log10(raw)))
    val n = raw / mag
    return mag * when {
        n < 1.5 -> 1.0
        n < 3 -> 2.0
        n < 7 -> 5.0
        else -> 10.0
    }
}

private fun tick(v: Double, step: Double): String = when {
    step >= 1 -> String.format(Locale.US, "%.0f", v)
    step >= 0.1 -> String.format(Locale.US, "%.1f", v)
    step >= 0.01 -> String.format(Locale.US, "%.2f", v)
    else -> String.format(Locale.US, "%.3f", v)
}

/** Scrolling time-series chart (the desktop StripChart). */
@Composable
fun StripChart(
    title: String,
    series: List<ChartSeries>,
    windowS: Int,
    now: Double,
    modifier: Modifier = Modifier,
    height: Dp = 150.dp,
    showLegend: Boolean = true,
) {
    val v = LocalVcu.current
    val tm = rememberTextMeasurer()
    Column(modifier.fillMaxWidth().padding(horizontal = 12.dp, vertical = 6.dp)) {
        Row(verticalAlignment = Alignment.CenterVertically) {
            Text(title, fontSize = 12.sp, color = v.ink2, modifier = Modifier.weight(1f), maxLines = 1, overflow = TextOverflow.Ellipsis)
            if (showLegend) for (s in series) if (s.label.isNotEmpty()) {
                Dot(s.color, 7.dp)
                Text(" ${s.label}  ", fontSize = 11.sp, color = v.ink3)
            }
        }
        Spacer(Modifier.height(4.dp))
        Canvas(Modifier.fillMaxWidth().height(height)) {
            val left = 46.dp.toPx()
            val bottom = 16.dp.toPx()
            val w = size.width - left - 4.dp.toPx()
            val h = size.height - bottom - 4.dp.toPx()
            val top = 4.dp.toPx()
            val t0 = now - windowS
            var lo = Double.POSITIVE_INFINITY
            var hi = Double.NEGATIVE_INFINITY
            for (s in series) for (i in 0 until s.data.size) {
                if (s.data.t[i] >= t0 - 1) {
                    val y = s.data.v[i]
                    if (y < lo) lo = y
                    if (y > hi) hi = y
                }
            }
            val labelStyle = TextStyle(fontSize = 9.5.sp, color = v.ink3, fontFamily = Mono)
            // frame
            drawRect(v.rule2, Offset(left, top), Size(w, h), style = Stroke(1f))
            // x grid
            val tstep = if (windowS <= 30) 5 else if (windowS <= 60) 10 else 30
            var s = 0
            while (s <= windowS) {
                val x = left + w - s.toFloat() / windowS * w
                drawLine(v.rule2, Offset(x, top), Offset(x, top + h), 1f)
                val txt = if (s == 0) "now" else "-${s}s"
                val lay = tm.measure(txt, labelStyle)
                drawText(lay, topLeft = Offset((x - lay.size.width / 2f).coerceIn(left, size.width - lay.size.width), top + h + 2f))
                s += tstep
            }
            if (lo == Double.POSITIVE_INFINITY) {
                val lay = tm.measure("Waiting for data", TextStyle(fontSize = 12.sp, color = v.ink3))
                drawText(lay, topLeft = Offset(left + w / 2 - lay.size.width / 2, top + h / 2 - lay.size.height / 2))
                return@Canvas
            }
            if (hi - lo < 1e-9) { lo -= 1; hi += 1 }
            val pad = (hi - lo) * 0.08
            lo -= pad; hi += pad
            val step = niceStep(hi - lo, 4)
            var g = ceil(lo / step) * step
            while (g <= hi) {
                val y = top + h - ((g - lo) / (hi - lo) * h).toFloat()
                drawLine(v.rule2, Offset(left, y), Offset(left + w, y), 1f)
                val lay = tm.measure(tick(g, step), labelStyle)
                drawText(lay, topLeft = Offset(left - lay.size.width - 4f, y - lay.size.height / 2))
                g += step
            }
            val stroke = Stroke(width = 1.6.dp.toPx(), cap = StrokeCap.Round, join = StrokeJoin.Round)
            for (sr in series) {
                val p = Path()
                var started = false
                for (i in 0 until sr.data.size) {
                    val t = sr.data.t[i]
                    if (t < t0 - 1) continue
                    val x = left + ((t - t0) / windowS * w).toFloat()
                    val y = top + h - ((sr.data.v[i] - lo) / (hi - lo) * h).toFloat()
                    if (!started) { p.moveTo(x.coerceAtLeast(left), y); started = true } else p.lineTo(x, y)
                }
                if (started) clipPath(Path().apply { addRect(androidx.compose.ui.geometry.Rect(left, top, left + w, top + h)) }) {
                    drawPath(p, sr.color, style = stroke)
                }
            }
        }
    }
}

/** Activity ribbon: one tick per message per lane (CAN1, CAN2, IMU, CSA, SYS) over the last 10 s. */
@Composable
fun Ribbon(hits: List<Pair<Int, Double>>, now: Double, modifier: Modifier = Modifier) {
    val v = LocalVcu.current
    val colors = listOf(v.can1, v.can2, v.imu, v.csa, v.sys)
    Canvas(modifier.fillMaxWidth().height(20.dp)) {
        val laneH = size.height / 5f
        for (i in 0 until 5) drawLine(v.rule2, Offset(0f, laneH * i + laneH / 2), Offset(size.width, laneH * i + laneH / 2), 1f)
        for ((lane, t) in hits) {
            val age = now - t
            if (age < 0 || age > 10) continue
            val x = size.width - (age / 10.0 * size.width).toFloat()
            val y = laneH * lane
            drawLine(
                colors[lane].copy(alpha = (1f - age.toFloat() / 12f).coerceIn(0.25f, 1f)),
                Offset(x, y + 0.5f), Offset(x, y + laneH - 0.5f), 2f,
            )
        }
    }
}

/** 240° arc gauge (speed / motor / SOC). */
@Composable
fun Gauge(e: VehEntry?, color: Color, modifier: Modifier = Modifier) {
    val v = LocalVcu.current
    val tm = rememberTextMeasurer()
    val value = e?.value
    val max = e?.max ?: 100.0
    val frac = if (value == null) 0f else (value / max).toFloat().coerceIn(0f, 1f)
    val anim = animateFloatAsState(frac, tween(350), label = "gauge").value
    Column(modifier, horizontalAlignment = Alignment.CenterHorizontally) {
        Canvas(Modifier.fillMaxWidth().aspectRatio(1.33f)) {
            val stroke = size.width * 0.07f
            val r = minOf(size.width / 2f, size.height * 0.62f) - stroke
            val c = Offset(size.width / 2f, size.height * 0.6f)
            val tl = Offset(c.x - r, c.y - r)
            val sz = Size(r * 2, r * 2)
            drawArc(v.rule2, 150f, 240f, false, tl, sz, style = Stroke(stroke, cap = StrokeCap.Round))
            if (anim > 0.001f) drawArc(color, 150f, 240f * anim, false, tl, sz, style = Stroke(stroke, cap = StrokeCap.Round))
            val txt = if (value == null) "–" else if (abs(value) >= 100) String.format(Locale.US, "%.0f", value) else String.format(Locale.US, "%.1f", value)
            val lay = tm.measure(txt, TextStyle(fontFamily = Mono, fontSize = (size.width / 7.5f).toSp(), fontWeight = FontWeight.Medium, color = v.ink))
            drawText(lay, topLeft = Offset(c.x - lay.size.width / 2, c.y - lay.size.height * 0.62f))
            val ul = tm.measure(e?.unit ?: "", TextStyle(fontSize = 11.sp, color = v.ink3))
            drawText(ul, topLeft = Offset(c.x - ul.size.width / 2, c.y + lay.size.height * 0.42f))
        }
        Text(e?.label ?: "", fontSize = 12.sp, color = v.ink2, maxLines = 1, overflow = TextOverflow.Ellipsis, textAlign = TextAlign.Center)
        Text(e?.signal ?: "Not mapped", fontSize = 10.sp, color = v.ink3, fontFamily = Mono, maxLines = 1, overflow = TextOverflow.Ellipsis)
    }
}

/** Artificial horizon from accelerometer roll/pitch. */
@Composable
fun Horizon(roll: Double, pitch: Double, modifier: Modifier = Modifier) {
    val v = LocalVcu.current
    val sky = if (v.dark) Color(0xFF2F5E8C) else Color(0xFF6FA8DC)
    val ground = if (v.dark) Color(0xFF6B4A2E) else Color(0xFFB08256)
    Box(modifier, contentAlignment = Alignment.Center) {
        Canvas(Modifier.width(220.dp).aspectRatio(1f)) {
            val k = size.width / 200f
            val c = Offset(size.width / 2, size.height / 2)
            val circle = Path().apply { addOval(androidx.compose.ui.geometry.Rect(c, 92 * k)) }
            clipPath(circle) {
                translate(c.x, c.y) {
                    rotate(-roll.toFloat(), pivot = Offset.Zero) {
                        translate(0f, (pitch * 2).toFloat().coerceIn(-80f, 80f) * k) {
                            drawRect(sky, Offset(-300 * k, -300 * k), Size(600 * k, 300 * k))
                            drawRect(ground, Offset(-300 * k, 0f), Size(600 * k, 300 * k))
                            drawLine(Color.White, Offset(-300 * k, 0f), Offset(300 * k, 0f), 1.5f * k)
                            for ((half, y) in listOf(20f to -30f, 20f to 30f, 12f to -15f, 12f to 15f)) {
                                drawLine(Color.White.copy(alpha = 0.8f), Offset(-half * k, y * k), Offset(half * k, y * k), 1.2f * k)
                            }
                        }
                    }
                }
            }
            drawCircle(v.rule, 92 * k, c, style = Stroke(3 * k))
            val plane = Path().apply {
                moveTo(c.x - 50 * k, c.y); lineTo(c.x - 20 * k, c.y); lineTo(c.x - 12 * k, c.y + 8 * k)
                lineTo(c.x - 4 * k, c.y); lineTo(c.x + 30 * k, c.y)
            }
            drawPath(plane, Color(0xFFF2C94C), style = Stroke(3.5f * k, cap = StrokeCap.Round, join = StrokeJoin.Round))
        }
    }
}
