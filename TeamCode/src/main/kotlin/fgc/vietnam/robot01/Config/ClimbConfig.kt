package fgc.vietnam.robot01.Config

import com.acmerobotics.dashboard.config.Config

@Config
object ClimbConfig {
    @JvmField var CUT_OFF_INPUT = 0.7

    @JvmField var TRIGGER_DEADBAND = 0.05;

    @JvmField var HOLD_STOP_POWER = 0.1;

    @JvmField var CLIMB_EXTEND_DISTANCE_MM = 300;

    @JvmField var HOLD_DELAY_CONFIRM_MS = 400;

    @JvmField var CLIMB_DEBUG = false;
}