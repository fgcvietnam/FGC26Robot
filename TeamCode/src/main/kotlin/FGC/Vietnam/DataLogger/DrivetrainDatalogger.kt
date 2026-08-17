package FGC.Vietnam.DataLogger

import FGC.Vietnam.Config.DrivetrainConfig
import com.qualcomm.robotcore.hardware.Gamepad
import fgc.vietnam.robot01.Hardware.DriveTelemetry
import java.io.File
import kotlin.math.abs

/**
 * Dedicated high-granularity datalogger for Drivetrain performance, SysId,
 * dynamics analysis, and back-and-forth automated tests.
 */
internal class DrivetrainDatalogger {

    private var logger: Datalogger? = null
    private var sessionStartNanos: Long = 0L
    private var lastRowNanos: Long = 0L
    private var previousYawRate: Double = 0.0
    private var previousLeftSpeed: Double = 0.0
    private var previousRightSpeed: Double = 0.0
    private var previousLinearSpeed: Double = 0.0
    private var rowIndex: Int = 0

    val isLogging: Boolean get() = logger != null
    val rowCount: Int get() = logger?.rowCount ?: 0
    val fileName: String? get() = logger?.file?.name
    val filePath: String? get() = logger?.file?.absolutePath
    val errorMessage: String? get() = logger?.errorMessage

    fun startLogging(prefix: String = "drivetrain_3m_test"): Boolean {
        if (logger == null) {
            val now = System.nanoTime()
            sessionStartNanos = now
            lastRowNanos = now
            previousYawRate = 0.0
            previousLeftSpeed = 0.0
            previousRightSpeed = 0.0
            previousLinearSpeed = 0.0
            rowIndex = 0
            logger = Datalogger.create(prefix = prefix, header = HEADER)
            return true
        }
        return false
    }

    fun stopLogging() {
        logger?.close()
        logger = null
    }

    fun writeRow(
        testPhase: String,
        cycleIndex: Int,
        totalCycles: Int,
        segmentIndex: Int,
        segmentDirection: String,
        segmentTargetDistanceM: Double,
        segmentProgressM: Double,
        segmentRemainingM: Double,
        totalDistanceM: Double,
        commandedForwardPwr: Double,
        commandedTurnPwr: Double,
        drive: DriveTelemetry,
        hubTemperaturesCelsius: List<Double> = emptyList(),
        gamepad1: Gamepad? = null,
    ) {
        val log = logger ?: return

        val nowNanos = System.nanoTime()
        val elapsedSec = (nowNanos - sessionStartNanos) / 1_000_000_000.0
        val deltaMs = (nowNanos - lastRowNanos) / 1_000_000.0
        val frequencyHz = if (deltaMs > 0.0) 1000.0 / deltaMs else 0.0
        lastRowNanos = nowNanos
        rowIndex++

        val dtSec = if (deltaMs > 0.0) deltaMs / 1000.0 else 0.001

        // Derived accelerations
        val headingAccelDegS2 = (drive.yawRate - previousYawRate) / dtSec
        previousYawRate = drive.yawRate

        val leftSpeedMs = drive.leftWheelSpeedMmPerSecond / 1000.0
        val rightSpeedMs = drive.rightWheelSpeedMmPerSecond / 1000.0
        val linearSpeedMs = drive.linearSpeedMmPerSecond / 1000.0

        val leftAccelMs2 = (leftSpeedMs - previousLeftSpeed) / dtSec
        val rightAccelMs2 = (rightSpeedMs - previousRightSpeed) / dtSec
        val linearAccelMs2 = (linearSpeedMs - previousLinearSpeed) / dtSec

        previousLeftSpeed = leftSpeedMs
        previousRightSpeed = rightSpeedMs
        previousLinearSpeed = linearSpeedMs

        // Derived RPMs
        val ticksPerRev = DrivetrainConfig.encoderTicksPerWheelRevolution
        val leftWheelRpm = if (ticksPerRev > 0) (drive.leftActualVelocity * 60.0) / ticksPerRev else 0.0
        val rightWheelRpm = if (ticksPerRev > 0) (drive.rightActualVelocity * 60.0) / ticksPerRev else 0.0
        val leftMotorRpm = leftWheelRpm * DrivetrainConfig.GEAR_REDUCTION
        val rightMotorRpm = rightWheelRpm * DrivetrainConfig.GEAR_REDUCTION

        // Electrical power estimation
        val leftPowerWatts = drive.batteryVoltage * drive.leftCurrentAmps
        val rightPowerWatts = drive.batteryVoltage * drive.rightCurrentAmps
        val totalPowerWatts = leftPowerWatts + rightPowerWatts

        val progressPct = if (segmentTargetDistanceM > 0) (segmentProgressM / segmentTargetDistanceM) * 100.0 else 0.0
        val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
        val leftWheelDistM = drive.leftEncoderPosition * (mmPerTick / 1000.0)
        val rightWheelDistM = drive.rightEncoderPosition * (mmPerTick / 1000.0)

        log.writeRow(
            listOf(
                // 1. Timing & Loop (5)
                "%.4f".format(elapsedSec),
                "%.3f".format(deltaMs),
                "%.1f".format(frequencyHz),
                System.currentTimeMillis(),
                rowIndex,

                // 2. Experiment / Test State (10)
                testPhase,
                cycleIndex,
                totalCycles,
                segmentIndex,
                segmentDirection,
                "%.4f".format(segmentTargetDistanceM),
                "%.4f".format(segmentProgressM),
                "%.2f".format(progressPct),
                "%.4f".format(segmentRemainingM),
                "%.4f".format(totalDistanceM),

                // 3. Drive Commands & Duty Cycle (5)
                "%.4f".format(commandedForwardPwr),
                "%.4f".format(commandedTurnPwr),
                "%.4f".format(drive.leftMotorPower),
                "%.4f".format(drive.rightMotorPower),
                "%.4f".format(drive.limitedForward),

                // 4. Encoders & Distances (5)
                drive.leftEncoderPosition,
                drive.rightEncoderPosition,
                "%.4f".format(leftWheelDistM),
                "%.4f".format(rightWheelDistM),
                "%.4f".format(leftWheelDistM - rightWheelDistM),

                // 5. Velocities, Speeds & Accelerations (17)
                "%.2f".format(drive.leftActualVelocity),
                "%.2f".format(drive.rightActualVelocity),
                "%.2f".format(drive.leftTargetVelocity),
                "%.2f".format(drive.rightTargetVelocity),
                "%.2f".format(drive.leftActualVelocity - drive.leftTargetVelocity),
                "%.2f".format(drive.rightActualVelocity - drive.rightTargetVelocity),
                "%.4f".format(leftSpeedMs),
                "%.4f".format(rightSpeedMs),
                "%.4f".format(linearSpeedMs),
                "%.4f".format(leftAccelMs2),
                "%.4f".format(rightAccelMs2),
                "%.4f".format(linearAccelMs2),
                "%.2f".format(leftMotorRpm),
                "%.2f".format(rightMotorRpm),
                "%.2f".format(leftWheelRpm),
                "%.2f".format(rightWheelRpm),
                "%.2f".format(abs(leftWheelRpm - rightWheelRpm)),

                // 6. Electrical & Temperatures (10)
                "%.4f".format(drive.batteryVoltage),
                "%.4f".format(drive.leftCurrentAmps),
                "%.4f".format(drive.rightCurrentAmps),
                "%.4f".format(drive.leftCurrentAmps + drive.rightCurrentAmps),
                "%.4f".format(abs(drive.leftCurrentAmps - drive.rightCurrentAmps)),
                "%.4f".format(leftPowerWatts),
                "%.4f".format(rightPowerWatts),
                "%.4f".format(totalPowerWatts),
                hubTemperaturesCelsius.fmtOrNan(0),
                hubTemperaturesCelsius.fmtOrNan(1),

                // 7. IMU & Heading PID (15)
                "%.4f".format(drive.heading),
                "%.4f".format(drive.targetHeading),
                "%.4f".format(drive.headingError),
                "%.4f".format(drive.pitchDegrees),
                "%.4f".format(drive.rollDegrees),
                "%.4f".format(drive.yawRate),
                "%.4f".format(drive.pitchRate),
                "%.4f".format(drive.rollRate),
                "%.4f".format(headingAccelDegS2),
                drive.headingHoldEnabled.i,
                "%.6f".format(drive.proportionalCorrection),
                "%.6f".format(drive.integralCorrection),
                "%.6f".format(drive.derivativeCorrection),
                "%.6f".format(drive.headingCorrection),
                drive.tiltingSafety.i,

                // 8. Localization & Pose (5)
                "%.2f".format(drive.poseX),
                "%.2f".format(drive.poseY),
                "%.2f".format(drive.poseHeading),
                drive.visionActive.i,
                drive.bestDetectionId,

                // 9. Gamepad Inputs (4)
                "%.4f".format(gamepad1?.left_stick_y ?: 0f),
                "%.4f".format(gamepad1?.right_stick_x ?: 0f),
                (gamepad1?.a ?: false).i,
                (gamepad1?.b ?: false).i,
            )
        )
    }

    private val Boolean.i: Int get() = if (this) 1 else 0
    private fun List<Double>.fmtOrNan(index: Int): String =
        if (index < size && !get(index).isNaN()) "%.2f".format(get(index)) else "NaN"

    companion object {
        val HEADER = listOf(
            // Timing & Loop (5)
            "elapsed_sec", "loop_dt_ms", "loop_hz", "absolute_timestamp_ms", "sample_index",

            // Experiment / Test State (10)
            "test_phase", "cycle_index", "total_cycles", "segment_index", "segment_direction",
            "segment_target_distance_m", "segment_progress_m", "segment_progress_pct", "segment_remaining_m", "total_distance_m",

            // Drive Commands & Duty Cycle (5)
            "commanded_forward_pwr", "commanded_turn_pwr", "left_motor_power", "right_motor_power", "limited_forward",

            // Encoders & Distances (5)
            "left_encoder_ticks", "right_encoder_ticks", "left_wheel_dist_m", "right_wheel_dist_m", "wheel_dist_diff_m",

            // Velocities, Speeds & Accelerations (17)
            "left_actual_vel_ticks_s", "right_actual_vel_ticks_s", "left_target_vel_ticks_s", "right_target_vel_ticks_s",
            "left_vel_error_ticks_s", "right_vel_error_ticks_s",
            "left_wheel_speed_m_s", "right_wheel_speed_m_s", "robot_linear_speed_m_s",
            "left_wheel_accel_m_s2", "right_wheel_accel_m_s2", "robot_linear_accel_m_s2",
            "left_motor_rpm", "right_motor_rpm", "left_wheel_rpm", "right_wheel_rpm", "wheel_rpm_diff",

            // Electrical & Temperatures (10)
            "battery_voltage_v", "left_current_a", "right_current_a", "total_current_a", "current_diff_a",
            "left_power_watts", "right_power_watts", "total_power_watts",
            "ctrl_hub_temp_c", "ext_hub_temp_c",

            // IMU & Heading PID (15)
            "heading_deg", "target_heading_deg", "heading_error_deg",
            "pitch_deg", "roll_deg",
            "yaw_rate_deg_s", "pitch_rate_deg_s", "roll_rate_deg_s", "heading_accel_deg_s2",
            "heading_hold_enabled", "heading_pid_p", "heading_pid_i", "heading_pid_d", "heading_pid_total",
            "tilting_safety",

            // Localization & Pose (5)
            "pose_x_mm", "pose_y_mm", "pose_heading_deg", "vision_active", "best_detection_id",

            // Gamepad Inputs (4)
            "gp1_left_stick_y", "gp1_right_stick_x", "gp1_a", "gp1_b"
        )
    }
}
