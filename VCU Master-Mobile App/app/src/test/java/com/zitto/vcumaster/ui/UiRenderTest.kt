package com.zitto.vcumaster.ui

import android.os.Looper
import androidx.compose.ui.test.junit4.createComposeRule
import androidx.compose.ui.test.onAllNodesWithText
import com.zitto.vcumaster.VcuApp
import com.zitto.vcumaster.ui.theme.VcuTheme
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.RuntimeEnvironment
import org.robolectric.Shadows.shadowOf
import org.robolectric.annotation.Config

/**
 * Composes and lays out every window of the real app (AppRoot + Hub + simulator) on the JVM and checks the
 * V2 content is on screen. Catches layout crashes in the new screens without a phone.
 */
@RunWith(RobolectricTestRunner::class)
@Config(sdk = [34], qualifiers = "w400dp-h880dp-xxhdpi", application = VcuApp::class)
class UiRenderTest {
    @get:Rule
    val rule = createComposeRule()

    private fun pump(ms: Long) {
        val end = System.currentTimeMillis() + ms
        while (System.currentTimeMillis() < end) {
            Thread.sleep(50)
            shadowOf(Looper.getMainLooper()).idle()
            rule.mainClock.advanceTimeBy(50)
        }
    }

    private fun seen(text: String) = rule.onAllNodesWithText(text, substring = true, useUnmergedTree = true).fetchSemanticsNodes().isNotEmpty()

    @Test
    fun rendersAllWindowsWithSimulator() {
        val app = RuntimeEnvironment.getApplication() as VcuApp
        val hub = app.hub
        val prefs = UiPrefs(app)
        rule.mainClock.autoAdvance = false
        rule.setContent { VcuTheme(dark = prefs.theme == "dark") { AppRoot(hub, prefs) } }
        hub.loadSampleDbc()
        hub.act { connect("SIM", null, false) }
        pump(6000)                      // simulator streams; the Daly BMS on CAN2 gets auto-matched from the library
        hub.module("IMU", true)         // command log + ACK
        pump(800)

        val st = hub.state.value
        assertTrue("sim connected", st.link.connected)
        assertTrue("Daly auto-loaded: ${st.dbcList.map { it.name }}", st.dbcList.any { it.name.startsWith("Daly") })
        assertTrue("battery panel", "battery" in st.roles.panels)
        assertTrue("cmd log ${st.cmdLog}", st.cmdLog.any { it.text.startsWith("S32K ACK MODULE IMU") })

        val expect = mapOf(
            Tab.CONNECT to listOf("Find a bridge", "Data integrity", "S32K frames received", "Bridge console"),
            Tab.LIVE to listOf("Showing", "matching", "MCU_Status"),
            Tab.BMS to listOf("Pack", "State of charge", "Cell voltages", "live of 16 shown", "Temperatures", "Faults and status"),
            Tab.MCU to listOf("Motor", "Drive, power and thermal"),
            Tab.VEHICLE to listOf("Decoded signals"),
            Tab.SENSORS to listOf("Board movement since IMU start", "Zero position", "Distance"),
            Tab.DEVICE to listOf("Command log", "S32K ACK MODULE IMU", "Modules", "GPIO"),
            Tab.UPDATES to listOf("S32K144 firmware update", "CAN databases", "DBC library", "Daly_BMS_CAN_V1.0_from_spec.dbc"),
        )
        val missing = ArrayList<String>()
        for (t in Tab.entries) {
            rule.runOnUiThread { prefs.open(t) }
            pump(700)
            if (!seen(t.title)) missing += "${t.key}: title"
            for (x in expect[t].orEmpty()) if (!seen(x)) missing += "${t.key}: $x"
        }
        rule.runOnUiThread { prefs.setView("table"); prefs.open(Tab.BMS) }
        pump(500)
        if (!seen("Signal")) missing += "bms table"
        rule.runOnUiThread { prefs.setThemeMode("dark"); prefs.setView("grid"); prefs.setLayout("24") }
        pump(500)
        if (!seen("not fitted")) missing += "bms 24-cell layout shows unfitted slots"
        rule.runOnUiThread { prefs.setThemeMode("system"); prefs.setLayout("auto") }
        hub.act { disconnect() }
        pump(300)
        assertTrue(missing.toString(), missing.isEmpty())
    }
}
