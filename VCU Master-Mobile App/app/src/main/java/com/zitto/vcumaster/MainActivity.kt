package com.zitto.vcumaster

import android.os.Bundle
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.enableEdgeToEdge
import androidx.compose.foundation.isSystemInDarkTheme
import com.zitto.vcumaster.ui.AppRoot
import com.zitto.vcumaster.ui.UiPrefs
import com.zitto.vcumaster.ui.theme.VcuTheme

class MainActivity : ComponentActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        enableEdgeToEdge()
        val hub = (application as VcuApp).hub
        val prefs = UiPrefs(applicationContext)
        setContent {
            val system = isSystemInDarkTheme()
            val dark = when (prefs.theme) {
                "dark" -> true
                "light" -> false
                else -> system
            }
            VcuTheme(dark) {
                AppRoot(hub, prefs)
            }
        }
    }
}
