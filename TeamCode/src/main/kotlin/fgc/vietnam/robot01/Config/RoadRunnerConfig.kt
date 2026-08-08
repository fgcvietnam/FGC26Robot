package fgc.vietnam.robot01.Config

import com.acmerobotics.dashboard.config.Config

@Config
object RoadRunnerConfig {
    // robot length: 46mm ~ 18.1102362205in
    const val FIELD_SIZE_IN = 275.59
    @JvmField var BLUE_STARTING_POSITION_X_IN = 0.0
    @JvmField var BLUE_STARTING_POSITION_Y_IN = 128.7398818898
    @JvmField var BLUE_STARTING_HEADING = 90.0

    @JvmField var RED_STARTING_POSITION_X_IN = 0.0
    @JvmField var RED_STARTING_POSITION_Y_IN = -128.7398818898
    @JvmField var RED_STARTING_HEADING = 270.0



}