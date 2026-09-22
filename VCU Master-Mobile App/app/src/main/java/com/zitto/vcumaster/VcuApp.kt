package com.zitto.vcumaster

import android.app.Application
import com.zitto.vcumaster.core.Hub

/** Holds the single [Hub] so the link and collected data survive rotation and screen changes. */
class VcuApp : Application() {
    lateinit var hub: Hub
        private set

    override fun onCreate() {
        super.onCreate()
        hub = Hub(this)
    }
}
