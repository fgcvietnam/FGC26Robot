package FGC.Vietnam.Config

import com.acmerobotics.dashboard.config.Config

@Config
object IntakeConfig {
    @JvmField var AUTO_EXTEND_TIME_SECONDS = 3

    @JvmField var INTAKE_HOMING_TIMEOUT_SECONDS = 5

    @JvmField var MOTOR_TEST_TIMEOUT_SECONDS = 5
    @JvmField var EXTENSION_HOMING = false
    @JvmField var ENABLE_LIMIT_SWITCH_AND_MAGNETIC_TESTING = false
    @JvmField var ENABLE_AUTO_EXTENDING = false
    @JvmField var MAGNETIC_SWITCH_CONFIRM_DELAY_MS = 200
    @JvmField var INTAKE_JAM_DETECTION_DELAY_MS = 400
    @JvmField var INTAKE_UNJAM_DELAY_MS = 100
    @JvmField var INTAKE_DEBUG = false
}