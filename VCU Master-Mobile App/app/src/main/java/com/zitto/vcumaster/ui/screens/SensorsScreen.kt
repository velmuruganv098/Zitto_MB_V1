package com.zitto.vcumaster.ui.screens

import androidx.compose.foundation.layout.fillMaxWidth
import androidx.compose.foundation.layout.padding
import androidx.compose.runtime.Composable
import androidx.compose.runtime.getValue
import androidx.compose.ui.Modifier
import androidx.compose.ui.unit.dp
import com.zitto.vcumaster.core.Fmt
import com.zitto.vcumaster.core.HubState
import com.zitto.vcumaster.core.SeriesSnap
import com.zitto.vcumaster.core.dbl
import com.zitto.vcumaster.core.str
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.components.ChartSeries
import com.zitto.vcumaster.ui.components.Horizon
import com.zitto.vcumaster.ui.components.KvList
import com.zitto.vcumaster.ui.components.Panel
import com.zitto.vcumaster.ui.components.Seg
import com.zitto.vcumaster.ui.components.StripChart
import com.zitto.vcumaster.ui.components.TileData
import com.zitto.vcumaster.ui.components.TileGrid
import com.zitto.vcumaster.ui.components.rememberNow
import com.zitto.vcumaster.ui.theme.LocalVcu

@Composable
fun SensorsScreen(st: HubState, prefs: UiPrefs) {
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
                else -> "Live, every 500 ms"
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
