package com.zitto.vcumaster.ui.theme

import androidx.compose.material3.MaterialTheme
import androidx.compose.material3.Typography
import androidx.compose.material3.darkColorScheme
import androidx.compose.material3.lightColorScheme
import androidx.compose.runtime.Composable
import androidx.compose.runtime.CompositionLocalProvider
import androidx.compose.runtime.Immutable
import androidx.compose.runtime.staticCompositionLocalOf
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.text.TextStyle
import androidx.compose.ui.text.font.FontFamily
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.sp

/** Bench-console palette from the desktop VCU Master (style.css). Channel colours are the one loud element. */
@Immutable
data class VcuColors(
    val dark: Boolean,
    val paper: Color, val panel: Color, val ink: Color, val ink2: Color, val ink3: Color,
    val rule: Color, val rule2: Color, val focus: Color, val ok: Color, val warn: Color, val err: Color,
    val imu: Color, val csa: Color, val can1: Color, val can2: Color, val flash: Color, val sys: Color,
    val gpio: Color, val ota: Color, val esp: Color, val tx: Color,
) {
    fun channel(ch: String): Color = when (ch) {
        "CAN1" -> can1; "CAN2" -> can2; "IMU" -> imu; "CSA" -> csa; "FLASH" -> flash
        "SYSTEM", "CMD", "LOG", "RAW" -> sys; "GPIO" -> gpio; "OTA" -> ota; "ESP32" -> esp; "TX" -> tx
        else -> ink3
    }
    val x get() = Color(0xFFCC3D33)
    val y get() = Color(0xFF1F8F5F)
    val z get() = Color(0xFF2A6FDB)
}

val LightVcu = VcuColors(
    dark = false,
    paper = Color(0xFFF3F5F4), panel = Color(0xFFFFFFFF), ink = Color(0xFF1D2733), ink2 = Color(0xFF4A5663),
    ink3 = Color(0xFF7B8692), rule = Color(0xFFD8DDE2), rule2 = Color(0xFFEAEDF0), focus = Color(0xFF2A6FDB),
    ok = Color(0xFF1F8F5F), warn = Color(0xFFB7791F), err = Color(0xFFCC3D33),
    imu = Color(0xFF0E9384), csa = Color(0xFFC9711A), can1 = Color(0xFF2A6FDB), can2 = Color(0xFF8A4FD3),
    flash = Color(0xFF6B7A2A), sys = Color(0xFF53606D), gpio = Color(0xFFB0447A), ota = Color(0xFFCC3D33),
    esp = Color(0xFF2B8AA8), tx = Color(0xFF1D2733),
)

val DarkVcu = VcuColors(
    dark = true,
    paper = Color(0xFF10151B), panel = Color(0xFF171E26), ink = Color(0xFFE3E8ED), ink2 = Color(0xFFAAB4BE),
    ink3 = Color(0xFF7C8792), rule = Color(0xFF2B3540), rule2 = Color(0xFF212A33), focus = Color(0xFF5A93F0),
    ok = Color(0xFF2FB37A), warn = Color(0xFFD9A441), err = Color(0xFFE5564B),
    imu = Color(0xFF2CC2AE), csa = Color(0xFFE8913A), can1 = Color(0xFF5A93F0), can2 = Color(0xFFAB7EF0),
    flash = Color(0xFFA4B347), sys = Color(0xFF9AA6B2), gpio = Color(0xFFDF6AA4), ota = Color(0xFFE5564B),
    esp = Color(0xFF4CB3D3), tx = Color(0xFFE3E8ED),
)

val LocalVcu = staticCompositionLocalOf { LightVcu }

val Mono = FontFamily.Monospace

val MonoSmall = TextStyle(fontFamily = Mono, fontSize = 12.sp, lineHeight = 16.sp)

@Composable
fun VcuTheme(dark: Boolean, content: @Composable () -> Unit) {
    val v = if (dark) DarkVcu else LightVcu
    val scheme = if (dark) {
        darkColorScheme(
            primary = v.ink, onPrimary = v.panel, secondary = v.imu, onSecondary = v.panel,
            background = v.paper, onBackground = v.ink, surface = v.panel, onSurface = v.ink,
            surfaceVariant = v.rule2, onSurfaceVariant = v.ink2, outline = v.rule, outlineVariant = v.rule2,
            error = v.err, surfaceContainer = v.panel, surfaceContainerHigh = Color(0xFF1C242E),
            surfaceContainerHighest = Color(0xFF232C37), surfaceContainerLow = v.panel,
            secondaryContainer = Color(0xFF243140), onSecondaryContainer = v.ink,
            primaryContainer = Color(0xFF243140), onPrimaryContainer = v.ink,
        )
    } else {
        lightColorScheme(
            primary = v.ink, onPrimary = v.panel, secondary = v.imu, onSecondary = v.panel,
            background = v.paper, onBackground = v.ink, surface = v.panel, onSurface = v.ink,
            surfaceVariant = v.rule2, onSurfaceVariant = v.ink2, outline = v.rule, outlineVariant = v.rule2,
            error = v.err, surfaceContainer = v.panel, surfaceContainerHigh = Color(0xFFF7F8F9),
            surfaceContainerHighest = Color(0xFFEFF2F4), surfaceContainerLow = v.panel,
            secondaryContainer = Color(0xFFE3E9EF), onSecondaryContainer = v.ink,
            primaryContainer = Color(0xFFE3E9EF), onPrimaryContainer = v.ink,
        )
    }
    val base = Typography()
    val typo = base.copy(
        titleLarge = base.titleLarge.copy(fontWeight = FontWeight.SemiBold, fontSize = 20.sp),
        titleMedium = base.titleMedium.copy(fontWeight = FontWeight.SemiBold, fontSize = 15.sp),
        titleSmall = base.titleSmall.copy(fontWeight = FontWeight.SemiBold, fontSize = 14.sp),
    )
    CompositionLocalProvider(LocalVcu provides v) {
        MaterialTheme(colorScheme = scheme, typography = typo, content = content)
    }
}
