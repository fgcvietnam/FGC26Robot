package fgc.vietnam.robot01.Config

import com.acmerobotics.dashboard.config.Config

@Config
object IntakeConfig {
    @JvmField var AUTO_EXTEND_TIME_SECONDS = 3

    @JvmField var INTAKE_HOMING_TIMEOUT_SECONDS = 5

    @JvmField var MOTOR_TEST_TIMEOUT_SECONDS = 5
    @JvmField var EXTENSION_HOMING = false
    @JvmField var ENABLE_LIMIT_SWITCH_AND_MAGNETIC_TESTING = true
    @JvmField var ENABLE_AUTO_EXTENDING = false
}