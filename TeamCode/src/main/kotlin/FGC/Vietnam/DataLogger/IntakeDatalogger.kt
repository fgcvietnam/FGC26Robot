package FGC.Vietnam.DataLogger

import FGC.Vietnam.Config.DrivetrainConfig
import com.qualcomm.robotcore.hardware.Gamepad
import fgc.vietnam.robot01.Hardware.DriveTelemetry
import fgc.vietnam.robot01.Hardware.Intake
import fgc.vietnam.robot01.Hardware.IntakeTelemetry

/**
 * Dedicated high-granularity datalogger for Intake automated tests,
 * measuring slide extension dynamics, magnet limit response, stiction,
 * intake motor current draw under load, electrical power, and chassis motion.
 */
internal class IntakeDatalogger {

    private var logger: Datalogger? = null
    private var sessionStartNanos: Long = 0L
    private var lastRowNanos: Long = 0L
    private var previousYawRate: Double = 0.0
    private var previousLinearSpeed: Double = 0.0
    private var rowIndex: Int = 0

    val isLogging: Boolean get() = logger != null
    val rowCount: Int get() = logger?.rowCount ?: 0
    val fileName: String? get() = logger?.file?.name
    val filePath: String? get() = logger?.file?.absolutePath
    val errorMessage: String? get() = logger?.errorMessage

    fun startLogging(prefix: String = "intake_3m_test"): Boolean {
        if (logger == null) {
            val now = System.nanoTime()
            sessionStartNanos = now
            lastRowNanos = now
            previousYawRate = 0.0
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
        intake: Intake,
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

        val linearSpeedMs = drive.linearSpeedMmPerSecond / 1000.0
        val linearAccelMs2 = (linearSpeedMs - previousLinearSpeed) / dtSec
        previousLinearSpeed = linearSpeedMs

        val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
        val leftWheelDistM = drive.leftEncoderPosition * (mmPerTick / 1000.0)
        val rightWheelDistM = drive.rightEncoderPosition * (mmPerTick / 1000.0)
        val progressPct = if (segmentTargetDistanceM > 0) (segmentProgressM / segmentTargetDistanceM) * 100.0 else 0.0

        val intakeTel: IntakeTelemetry = intake.getTelemetry()

        // Power calculations
        val intakeMotorWatts = drive.batteryVoltage * intakeTel.motorCurrentAmps
        val intakeHexWatts = drive.batteryVoltage * intakeTel.hexMotorCurrentAmps
        val totalIntakeWatts = intakeMotorWatts + intakeHexWatts

        val driveLeftWatts = drive.batteryVoltage * drive.leftCurrentAmps
        val driveRightWatts = drive.batteryVoltage * drive.rightCurrentAmps
        val totalDriveWatts = driveLeftWatts + driveRightWatts

        val totalRobotCurrentA = drive.leftCurrentAmps + drive.rightCurrentAmps + intakeTel.motorCurrentAmps + intakeTel.hexMotorCurrentAmps
        val totalRobotPowerWatts = drive.batteryVoltage * totalRobotCurrentA

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

                // 3. Intake State & Actuators (11)
                intakeTel.state.name,
                intakeTel.leftSlidePosition.name,
                intakeTel.rightSlidePosition.name,
                intakeTel.slidesSynchronized.i,
                intakeTel.slideSkewState,
                "%.4f".format(intakeTel.motorPower),
                "%.4f".format(intakeTel.hexMotorPower),
                "%.4f".format(intakeTel.servoLeftBelowPower),
                "%.4f".format(intakeTel.servoRightBelowPower),
                "%.4f".format(intakeTel.servoLeftAbovePower),
                "%.4f".format(intakeTel.servoRightAbovePower),

                // 4. Intake Sensors & Jam Protection (13)
                intakeTel.leftLimitSwitchPressed.i,
                intakeTel.rightLimitSwitchPressed.i,
                intakeTel.bothLimitsPressed.i,
                intakeTel.limitSyncState,
                intakeTel.leftMagneticSwitchPressed.i,
                intakeTel.rightMagneticSwitchPressed.i,
                intakeTel.bothMagnetsDetected.i,
                intakeTel.magnetSyncState,
                intakeTel.leftMagnetConfirmed.i,
                intakeTel.rightMagnetConfirmed.i,
                intakeTel.bothMagnetsConfirmed.i,
                intakeTel.intakeJamConfirmed.i,
                intakeTel.transferJamConfirmed.i,

                // 5. Intake Electrical (5)
                "%.4f".format(intakeTel.motorCurrentAmps),
                "%.4f".format(intakeTel.hexMotorCurrentAmps),
                "%.4f".format(intakeMotorWatts),
                "%.4f".format(intakeHexWatts),
                "%.4f".format(totalIntakeWatts),

                // 6. Drivetrain Commands & Duty Cycle (5)
                "%.4f".format(commandedForwardPwr),
                "%.4f".format(commandedTurnPwr),
                "%.4f".format(drive.leftMotorPower),
                "%.4f".format(drive.rightMotorPower),
                "%.4f".format(drive.limitedForward),

                // 7. Drivetrain Encoders & Kinematics (9)
                drive.leftEncoderPosition,
                drive.rightEncoderPosition,
                "%.4f".format(leftWheelDistM),
                "%.4f".format(rightWheelDistM),
                "%.2f".format(drive.leftActualVelocity),
                "%.2f".format(drive.rightActualVelocity),
                "%.4f".format(linearSpeedMs),
                "%.4f".format(linearAccelMs2),
                "%.4f".format(drive.linearSpeedMmPerSecond),

                // 8. Robot Electrical & Thermal (8)
                "%.4f".format(drive.batteryVoltage),
                "%.4f".format(drive.leftCurrentAmps),
                "%.4f".format(drive.rightCurrentAmps),
                "%.4f".format(drive.leftCurrentAmps + drive.rightCurrentAmps),
                "%.4f".format(totalDriveWatts),
                "%.4f".format(totalRobotCurrentA),
                "%.4f".format(totalRobotPowerWatts),
                hubTemperaturesCelsius.fmtOrNan(0),

                // 9. Heading & IMU Dynamics (10)
                "%.4f".format(drive.heading),
                "%.4f".format(drive.targetHeading),
                "%.4f".format(drive.headingError),
                "%.4f".format(drive.pitchDegrees),
                "%.4f".format(drive.rollDegrees),
                "%.4f".format(drive.yawRate),
                "%.4f".format(headingAccelDegS2),
                drive.headingHoldEnabled.i,
                "%.6f".format(drive.proportionalCorrection),
                "%.6f".format(drive.derivativeCorrection),
            )
        )
    }

    private val Boolean.i: Int get() = if (this) 1 else 0

    private fun List<Double>.fmtOrNan(index: Int): String =
        if (index < size && !get(index).isNaN()) "%.2f".format(get(index)) else "NaN"

    companion object {
        val HEADER = listOf(
            // 1. Timing & Loop (5)
            "elapsed_sec",
            "loop_dt_ms",
            "loop_hz",
            "absolute_timestamp_ms",
            "sample_index",

            // 2. Experiment / Test State (10)
            "test_phase",
            "cycle_index",
            "total_cycles",
            "segment_index",
            "segment_direction",
            "segment_target_distance_m",
            "segment_progress_m",
            "segment_progress_pct",
            "segment_remaining_m",
            "total_distance_m",

            // 3. Intake State & Actuators (11)
            "intake_state",
            "left_slide_position",
            "right_slide_position",
            "slides_synchronized",
            "slide_skew_state",
            "intake_motor_power",
            "intake_hex_motor_power",
            "servo_left_below_power",
            "servo_right_below_power",
            "servo_left_above_power",
            "servo_right_above_power",

            // 4. Intake Sensors & Jam Protection (13)
            "left_limit_switch_pressed",
            "right_limit_switch_pressed",
            "both_limits_pressed",
            "limit_sync_state",
            "left_magnetic_switch_pressed",
            "right_magnetic_switch_pressed",
            "both_magnets_detected",
            "magnet_sync_state",
            "left_magnet_confirmed",
            "right_magnet_confirmed",
            "both_magnets_confirmed",
            "intake_jam_confirmed",
            "transfer_jam_confirmed",

            // 5. Intake Electrical (5)
            "intake_motor_current_a",
            "intake_hex_current_a",
            "intake_motor_watts",
            "intake_hex_watts",
            "total_intake_watts",

            // 6. Drivetrain Commands & Duty Cycle (5)
            "commanded_forward_pwr",
            "commanded_turn_pwr",
            "left_motor_power",
            "right_motor_power",
            "limited_forward",

            // 7. Drivetrain Encoders & Kinematics (9)
            "left_encoder_ticks",
            "right_encoder_ticks",
            "left_wheel_dist_m",
            "right_wheel_dist_m",
            "left_actual_vel_ticks_s",
            "right_actual_vel_ticks_s",
            "robot_linear_speed_m_s",
            "robot_linear_accel_m_s2",
            "robot_linear_speed_mm_s",

            // 8. Robot Electrical & Thermal (8)
            "battery_voltage_v",
            "left_drive_current_a",
            "right_drive_current_a",
            "total_drive_current_a",
            "total_drive_watts",
            "total_robot_current_a",
            "total_robot_power_watts",
            "ctrl_hub_temp_c",

            // 9. Heading & IMU Dynamics (10)
            "heading_deg",
            "target_heading_deg",
            "heading_error_deg",
            "pitch_deg",
            "roll_deg",
            "yaw_rate_deg_s",
            "heading_accel_deg_s2",
            "heading_hold_enabled",
            "heading_pid_p",
            "heading_pid_d",
        )
    }
}
