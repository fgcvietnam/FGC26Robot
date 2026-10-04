package FGC.Vietnam.Config

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI

@Config
internal object FlywheelConfig {
    const val LEFT_SHOOTER_MOTOR = "leftShooterMotor"
    const val RIGHT_SHOOTER_MOTOR = "rightShooterMotor"
    @JvmField var DEFAULT_RPM = 1200.0
    @JvmField var PIDF_P = 0.001
    @JvmField var PIDF_I = 0.0
    @JvmField var PIDF_D = 0.00001

    @JvmField var FF_KS = 0.0
    @JvmField var FF_KV = 0.000425
    @JvmField var FF_KA = 0.0

    @JvmField var ENABLE_PIDF_TUNING = false


    const val ENCODER_MIN_VELOCITY = 100.0
    const val ENCODER_MISMATCH_RATIO = 0.10
    const val ENCODER_MISMATCH_TIME_MS = 250L

    const val READY_ERROR_RATIO = 0.05

    const val MIN_READY_ERROR_RPM = 100.0
}