package FGC.Vietnam.Config

import com.acmerobotics.dashboard.config.Config
import kotlin.math.PI

@Config
internal object DrivetrainConfig {
    const val GEAR_REDUCTION = 13.0975
    const val MOTOR_ENCODER_TICKS_PER_REVOLUTION = 28.0
    const val WHEEL_DIAMETER_MM = 90.0
    const val TRACK_WIDTH_MM = 423.0

    const val JOYSTICK_DEADBAND = 0.05

    val encoderTicksPerWheelRevolution = MOTOR_ENCODER_TICKS_PER_REVOLUTION * GEAR_REDUCTION
    val millimetersPerEncoderTick = PI * WHEEL_DIAMETER_MM / encoderTicksPerWheelRevolution
    // Drive shaping and limits
    @JvmField var DATALOG_ENABLED = false
    @JvmField var FORWARD_DEADBAND = 0.05
    @JvmField var FORWARD_EXPO = 0.25
    @JvmField var TURN_DEADBAND = 0.05
    @JvmField var TURN_EXPO = 0.25
    @JvmField var DRIVE_SPEED_MULTIPLIER = 1.0
    @JvmField var TURN_SPEED_MULTIPLIER = 1.0
    @JvmField var PRECISION_MODE_MULTIPLIER = 0.3
    @JvmField var MIN_DRIVE_POWER = 0.05

    // Turn boost
    @JvmField var TURN_BOOST_START = 0.4
    @JvmField var TURN_BOOST_END = 0.8
    @JvmField var HIGH_SPEED_TURN_BOOST = 1.8
    @JvmField var TURN_BOOST_EXPONENT = 2.0

    // Voltage and Smart Power
    @JvmField var MIN_VOLTAGE_COMPENSATION = 0.8
    @JvmField var MAX_VOLTAGE_COMPENSATION = 1.2
    @JvmField var MOTOR_POWER_TOLERANCE = 0.01

    // Active Heading Correction
    @JvmField var ENABLE_ACTIVE_HEADING_TUNING = false
    @JvmField var ENABLE_ACTIVE_HEADING_CORRECTION = true

    @JvmField var LOCALIZER_ENABLE = true
    @JvmField var ACTIVE_HEADING_KP = 3.0
    @JvmField var ACTIVE_HEADING_KI = 2.0
    @JvmField var ACTIVE_HEADING_KD = 0.4
    @JvmField var ACTIVE_HEADING_KS = 0.1
    @JvmField var ACTIVE_HEADING_HOLD_DEADBAND_DEG = 1.0
    @JvmField var ACTIVE_HEADING_TURN_DEADBAND = 0.05
    @JvmField var ACTIVE_HEADING_SETTLE_TIME_SECONDS = 0.25
    @JvmField var MAX_TILT_FOR_HEADING_CORRECTION_DEG = 15.0
    @JvmField var DPAD_ORIENTATION_TIMEOUT_SECONDS = -1.0 // Disabled

    const val LEFT_MOTOR_NAME = "driveLeft"
    const val RIGHT_MOTOR_NAME = "driveRight"
    const val IMU_NAME = "imu"
}