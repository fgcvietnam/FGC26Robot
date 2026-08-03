package fgc.vietnam.robot01

import kotlin.math.PI

internal object FlywheelConfig {
    const val PRIMARY_MOTOR_NAME = "flywheel"
    const val SECONDARY_MOTOR_NAME = "flywheel_2"
    const val GEAR_REDUCTION = 1.0
    const val MOTOR_ENCODER_TICKS_PER_REVOLUTION = 28.0
    const val WHEEL_DIAMETER_MM = 90.0
    const val DEFAULT_RPM = 4_000.0

    // Tuned PIDF for 6000 max RPM / 2800 ticks/sec
    const val PIDF_P = 2.5
    const val PIDF_I = 0.0
    const val PIDF_D = 0.0
    const val PIDF_F = 11.7

    val encoderTicksPerWheelRevolution = MOTOR_ENCODER_TICKS_PER_REVOLUTION * GEAR_REDUCTION

    fun rpmToTicksPerSecond(rpm: Double) = rpm * encoderTicksPerWheelRevolution / 60.0
    fun ticksPerSecondToRpm(ticksPerSecond: Double) = ticksPerSecond * 60.0 / encoderTicksPerWheelRevolution
    fun rpmToSurfaceSpeedMetersPerSecond(rpm: Double) = rpm / 60.0 * PI * WHEEL_DIAMETER_MM / 1_000.0
}
