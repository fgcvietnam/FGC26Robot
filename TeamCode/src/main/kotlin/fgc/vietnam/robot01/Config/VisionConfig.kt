package fgc.vietnam.robot01.Config

import com.acmerobotics.dashboard.config.Config

@Config
object VisionConfig {
    @JvmField var MAX_VISION_ERROR: Double = 18.0
    @JvmField var VISION_BLEND: Double = 0.15
    @JvmField var ALPHA_FAR: Double = 0.05
    @JvmField var ALPHA_CLOSE: Double = 0.3


}