package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.Canvas
import androidx.compose.foundation.layout.aspectRatio
import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.geometry.Offset
import androidx.compose.ui.graphics.Path
import androidx.compose.ui.graphics.drawscope.Stroke
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.drawText
import androidx.compose.ui.text.rememberTextMeasurer
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.Hub
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.core.SeriesSnap
import com.zitto.vcumaster.core.dbl
import com.zitto.vcumaster.core.long
import com.zitto.vcumaster.core.str
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.ChartSeries
import com.zitto.vcumaster.ui.components.Horizon
import com.zitto.vcumaster.ui.components.KvList
import com.zitto.vcumaster.ui.components.Note
import com.zitto.vcumaster.ui.components.SmallButton
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.components.StripChart
import com.zitto.vcumaster.ui.components.TileData
import com.zitto.vcumaster.ui.components.TileGrid
import com.zitto.vcumaster.ui.components.rememberNow
import com.zitto.vcumaster.ui.theme.LocalVcu

@Composable
fun SensorsScreen(st: HubState, hub: Hub, prefs: UiPrefs) {
    val v = LocalVcu.current
    val now by rememberNow(100)
    val L = st.latest
    val imu = L.imu
    val csa = L.csa
    val staleI = L.imuT == null || now - L.imuT > 3
    val staleC = L.csaT == null || now - L.csaT > 3
    fun s(k: String) = st.charts[k] ?: SeriesSnap.EMPTY
    val win = prefs.winS

    ScreenColumn {
        Panel("Chart window") {
            Seg(
                listOf(15 to "15 s", 30 to "30 s", 60 to "60 s", 120 to "2 min"), win, { prefs.setWindow(it) },
                Modifier.fillMaxWidth().padding(start = 12.dp, end = 12.dp, bottom = 10.dp),
            )
        }

        Panel(
            "IMU (ICM-42670-P)", accent = v.imu,
            sub = when {
                imu == null -> "No IMU data yet. Enable the IMU module on the Device tab."
                staleI -> "Last sample ${Fmt.ago(L.imuT)} ago"
                else -> "Live, every 100 ms"
            },
        ) {
            if (imu != null) {
                TileGrid(
                    listOf(
                        TileData("Accel X", imu.str("ax_mg") ?: "–", "mg", staleI),
                        TileData("Accel Y", imu.str("ay_mg") ?: "–", "mg", staleI),
                        TileData("Accel Z", imu.str("az_mg") ?: "–", "mg", staleI),
                        TileData("|a|", Fmt.fix(imu.dbl("accel_g"), 3), "g", staleI),
                        TileData("Gyro X", Fmt.fix(imu.dbl("gx_mdps") / 1000, 2), "dps", staleI),
                        TileData("Gyro Y", Fmt.fix(imu.dbl("gy_mdps") / 1000, 2), "dps", staleI),
                        TileData("Gyro Z", Fmt.fix(imu.dbl("gz_mdps") / 1000, 2), "dps", staleI),
                        TileData("Die temp", imu.str("temp_c") ?: "–", "°C", staleI),
                    ),
                    columns = 4,
                )
            }
            val xyz = { a: String, b: String, c: String ->
                listOf(ChartSeries("X", v.x, s(a)), ChartSeries("Y", v.y, s(b)), ChartSeries("Z", v.z, s(c)))
            }
            StripChart("Acceleration (mg)", xyz("ax", "ay", "az"), win, now)
            StripChart("Angular rate (dps)", xyz("gx", "gy", "gz"), win, now)
        }

        Panel("Attitude", sub = "From accelerometer", accent = v.imu) {
            Horizon(imu?.dbl("roll_deg") ?: 0.0, imu?.dbl("pitch_deg") ?: 0.0, Modifier.fillMaxWidth().padding(8.dp))
            KvList(
                listOf(
                    "Roll" to (imu?.let { Fmt.fix(it.dbl("roll_deg"), 1) + "°" } ?: "–"),
                    "Pitch" to (imu?.let { Fmt.fix(it.dbl("pitch_deg"), 1) + "°" } ?: "–"),
                    "MCU timestamp" to (imu?.let { Fmt.dur(it.dbl("ts_ms")) } ?: "–"),
                ),
                columns = 3,
            )
        }

        MovementPanel(st, hub)

        Panel(
            "Current sense amplifier", accent = v.csa,
            sub = when {
                csa == null -> "No CSA data yet"
                staleC -> "Last sample ${Fmt.ago(L.csaT)} ago"
                else -> "Live, every 200 ms"
            },
        ) {
            if (csa != null) {
                TileGrid(
                    listOf(
                        TileData("Current", csa.str("current_ma") ?: "–", "mA", staleC),
                        TileData("Voltage", Fmt.fix(csa.dbl("voltage_mv") / 1000, 3), "V", staleC),
                        TileData("Power", Fmt.fix(csa.dbl("power_mw") / 1000, 3), "W", staleC),
                        TileData("Avg current 10 s", st.csaAvg10?.let { Fmt.fix(it, 0) } ?: "–", "mA", staleC),
                        TileData("Peak 10 s", st.csaPeak10?.let { Fmt.fix(it, 0) } ?: "–", "mA", staleC),
                        TileData("MCU timestamp", Fmt.dur(csa.dbl("ts_ms")), "", staleC),
                    ),
                    columns = 3,
                )
            }
            StripChart("Current (mA)", listOf(ChartSeries("", v.csa, s("cur"))), win, now, height = 120.dp)
            StripChart("Voltage (mV)", listOf(ChartSeries("", v.csa, s("vol"))), win, now, height = 120.dp)
            StripChart("Power (mW)", listOf(ChartSeries("", v.csa, s("pow"))), win, now, height = 120.dp)
        }
    }
}

/** V0.0073: board displacement since the IMU started (or the last CMD_IMU_ZERO) and the X/Y path. */
@Composable
private fun MovementPanel(st: HubState, hub: Hub) {
    val v = LocalVcu.current
    val imu = st.latest.imu
    val has = imu != null && imu.containsKey("pos_x_mm")
    Panel(
        "Board movement since IMU start", accent = v.imu,
        sub = if (has) "${if (imu!!.long("moving") != 0L) "MOVING" else "STILL"} · tracking for ${fmtDur(imu.long("imu_up_ms"))}" else null,
        actions = { SmallButton("Zero position") { hub.imuZero() } },
    ) {
        if (!has) {
            Note("Needs S32K firmware V0.0073 or newer (IMU displacement fields).")
            return@Panel
        }
        val m = imu!!
        TileGrid(
            listOf(
                TileData("X", Fmt.fix(m.dbl("pos_x_mm"), 1), "mm"),
                TileData("Y", Fmt.fix(m.dbl("pos_y_mm"), 1), "mm"),
                TileData("Z (up)", Fmt.fix(m.dbl("pos_z_mm"), 1), "mm"),
                TileData("Distance", Fmt.fix(m.dbl("dist_mm"), 1), "mm"),
                TileData("Speed", Fmt.fix(m.dbl("speed_mms"), 1), "mm/s"),
                TileData("Roll", Fmt.fix(m.dbl("roll_fw"), 1), "°"),
                TileData("Pitch", Fmt.fix(m.dbl("pitch_fw"), 1), "°"),
                TileData("Yaw", Fmt.fix(m.dbl("yaw_fw"), 1), "°"),
            ),
            columns = 4,
        )
        MoveTrack(st.track, Modifier.fillMaxWidth().padding(horizontal = 40.dp, vertical = 6.dp))
        Note(
            "Double-integrated accelerometer with orientation tracking and zero-velocity updates: short moves with pauses " +
                "are tracked to about centimetre level; error grows the longer the board keeps moving without a pause.",
        )
    }
}

private fun fmtDur(ms: Long): String {
    if (ms <= 0) return "0 s"
    val s = ms / 1000
    val h = s / 3600
    val m = (s % 3600) / 60
    return if (h > 0) "$h h $m min" else if (m > 0) "$m min ${s % 60} s" else "$s s"
}

/** Top view of the X/Y path since the last zero, auto-scaled (at least ±50 mm). */
@Composable
private fun MoveTrack(track: List<Pair<Double, Double>>, modifier: Modifier) {
    val v = LocalVcu.current
    val tm = rememberTextMeasurer()
    Canvas(modifier.aspectRatio(1f)) {
        val c = Offset(size.width / 2, size.height / 2)
        val k = size.width / 200f
        drawLine(v.rule, Offset(5 * k, c.y), Offset(195 * k, c.y), 1f)
        drawLine(v.rule, Offset(c.x, 5 * k), Offset(c.x, 195 * k), 1f)
        var span = 50.0
        for ((x, y) in track) span = maxOf(span, kotlin.math.abs(x), kotlin.math.abs(y))
        span *= 1.15
        fun pt(x: Double, y: Double) = Offset(c.x + (x / span * 90 * k).toFloat(), c.y - (y / span * 90 * k).toFloat())
        val lb = TextStyle(fontSize = 10.sp, color = v.ink3)
        drawText(tm.measure("+X", lb), topLeft = Offset(185 * k, c.y - 16 * k))
        drawText(tm.measure("+Y", lb), topLeft = Offset(c.x + 4 * k, 4 * k))
        drawText(tm.measure("scale ±${span.toInt()} mm", lb), topLeft = Offset(4 * k, 186 * k))
        if (track.size > 1) {
            val path = Path()
            track.forEachIndexed { i, (x, y) -> val p = pt(x, y); if (i == 0) path.moveTo(p.x, p.y) else path.lineTo(p.x, p.y) }
            drawPath(path, v.imu, style = Stroke(2f * k / 1.2f))
        }
        track.lastOrNull()?.let { (x, y) -> drawCircle(v.err, 3.5f * k, pt(x, y)) }
    }
}
