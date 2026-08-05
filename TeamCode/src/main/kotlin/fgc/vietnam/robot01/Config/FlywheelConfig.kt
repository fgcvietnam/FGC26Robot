package fgc.vietnam.robot01.Config

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI

@Config
internal object FlywheelConfig {
    const val LEFT_SHOOTER_MOTOR = "leftShooterMotor"
    const val RIGHT_SHOOTER_MOTOR = "rightShooterMotor"
    const val GEAR_REDUCTION = 1.0
    const val MOTOR_ENCODER_TICKS_PER_REVOLUTION = 28.0
    const val WHEEL_DIAMETER_MM = 90.0
    @JvmField var DEFAULT_RPM = 1_800.0

    @JvmField var PIDF_P = 0.001
    @JvmField var PIDF_I = 0.0
    @JvmField var PIDF_D = 0.00001

    @JvmField var FF_KS = 0.0
    @JvmField var FF_KV = 0.000425
    @JvmField var FF_KA = 0.0

    @JvmField var ENABLE_PIDF_TUNING = false

    @JvmField var DATALOG_ENABLED = false
    val encoderTicksPerWheelRevolution = MOTOR_ENCODER_TICKS_PER_REVOLUTION * GEAR_REDUCTION

    fun rpmToTicksPerSecond(rpm: Double) = rpm * encoderTicksPerWheelRevolution / 60.0
    fun ticksPerSecondToRpm(ticksPerSecond: Double) = ticksPerSecond * 60.0 / encoderTicksPerWheelRevolution
    fun rpmToSurfaceSpeedMetersPerSecond(rpm: Double) = rpm / 60.0 * PI * WHEEL_DIAMETER_MM / 1_000.0
}