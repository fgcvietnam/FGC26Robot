package fgc.vietnam.robot01

import com.qualcomm.robotcore.hardware.Gamepad

/**
 * Full-robot TeleOp CSV datalogger — 104 columns per row.
 *
 * Column groups:
 *   - Timing      : elapsed_sec, loop_delta_ms, absolute_timestamp_ms, row_index
 *   - Gamepad 1   : 6 axes + 14 buttons
 *   - Gamepad 2   : 6 axes + 14 buttons
 *   - Hub temps   : Control Hub & Extension Hub chip temperature (°C)
 *   - Drive       : inputs, full heading PID, velocities, velocity errors,
 *                   individual wheel speeds, encoder positions,
 *                   motor powers, currents, yaw/pitch/roll + rates,
 *                   heading angular acceleration, battery voltage
 *   - Flywheel    : summary + per-motor RPM, RPM error, velocity,
 *                   current, encoder position, duty-cycle power
 *   - Intake      : hex-mode flag, motor/servo commanded powers
 *
 * Toggle with toggleLogging().  Flushed every 25 rows; closed on stop().
 * File → /sdcard/FIRST/Datalogs/teleop_full_<timestamp>.csv
 */
internal class TeleOpDatalogger {

    private var logger: Datalogger? = null
    private var sessionStartNanos: Long = 0L
    private var lastRowNanos: Long = 0L
    private var previousYawRate: Double = 0.0
    private var rowIndex: Int = 0

    val isLogging: Boolean get() = logger != null
    val rowCount: Int get() = logger?.rowCount ?: 0

    /** Toggle start / stop. Returns true if logging just started. */
    fun toggleLogging(): Boolean {
        return if (logger == null) {
            val now = System.nanoTime()
            sessionStartNanos = now
            lastRowNanos = now
            previousYawRate = 0.0
            rowIndex = 0
            logger = Datalogger.create(prefix = "teleop_full", header = HEADER)
            true
        } else {
            stopLogging()
            false
        }
    }

    fun writeRow(
        gamepad1: Gamepad,
        gamepad2: Gamepad,
        drive: DriveTelemetry,
        flywheel: FlywheelTelemetry,
        intake: Intake,
        hubTemperaturesCelsius: List<Double>,
    ) {
        val log = logger ?: return

        // ── Timing ────────────────────────────────────────────────────────────
        val nowNanos = System.nanoTime()
        val elapsedSec = (nowNanos - sessionStartNanos) / 1_000_000_000.0
        val deltaMs    = (nowNanos - lastRowNanos)      /     1_000_000.0
        lastRowNanos = nowNanos
        rowIndex++

        // ── Derived: heading angular acceleration (deg/s²) ────────────────────
        val headingAccelDegS2 = if (deltaMs > 0.0) {
            (drive.yawRate - previousYawRate) / (deltaMs / 1_000.0)
        } else {
            0.0
        }
        previousYawRate = drive.yawRate

        // ── Derived: flywheel RPM errors ──────────────────────────────────────
        val fwPrimaryRpmError   = flywheel.primaryMotor.rpm   - flywheel.targetRpm
        val fwSecondaryRpmError = flywheel.secondaryMotor.rpm - flywheel.targetRpm

        log.writeRow(
            listOf(
                // ── Timing ────────────────────────────────────────────────────
                "%.4f".format(elapsedSec),
                "%.3f".format(deltaMs),
                System.currentTimeMillis(),
                rowIndex,

                // ── Gamepad 1 axes ─────────────────────────────────────────────
                "%.4f".format(gamepad1.left_stick_x),
                "%.4f".format(gamepad1.left_stick_y),
                "%.4f".format(gamepad1.right_stick_x),
                "%.4f".format(gamepad1.right_stick_y),
                "%.4f".format(gamepad1.left_trigger),
                "%.4f".format(gamepad1.right_trigger),

                // ── Gamepad 1 buttons ──────────────────────────────────────────
                gamepad1.a.i,
                gamepad1.b.i,
                gamepad1.x.i,
                gamepad1.y.i,
                gamepad1.left_bumper.i,
                gamepad1.right_bumper.i,
                gamepad1.dpad_up.i,
                gamepad1.dpad_down.i,
                gamepad1.dpad_left.i,
                gamepad1.dpad_right.i,
                gamepad1.start.i,
                gamepad1.back.i,
                gamepad1.left_stick_button.i,
                gamepad1.right_stick_button.i,

                // ── Gamepad 2 axes ─────────────────────────────────────────────
                "%.4f".format(gamepad2.left_stick_x),
                "%.4f".format(gamepad2.left_stick_y),
                "%.4f".format(gamepad2.right_stick_x),
                "%.4f".format(gamepad2.right_stick_y),
                "%.4f".format(gamepad2.left_trigger),
                "%.4f".format(gamepad2.right_trigger),

                // ── Gamepad 2 buttons ──────────────────────────────────────────
                gamepad2.a.i,
                gamepad2.b.i,
                gamepad2.x.i,
                gamepad2.y.i,
                gamepad2.left_bumper.i,
                gamepad2.right_bumper.i,
                gamepad2.dpad_up.i,
                gamepad2.dpad_down.i,
                gamepad2.dpad_left.i,
                gamepad2.dpad_right.i,
                gamepad2.start.i,
                gamepad2.back.i,
                gamepad2.left_stick_button.i,
                gamepad2.right_stick_button.i,

                // ── REV Hub temperatures ───────────────────────────────────────
                hubTemperaturesCelsius.fmtOrNan(0),
                hubTemperaturesCelsius.fmtOrNan(1),

                // ── Drivetrain inputs ──────────────────────────────────────────
                "%.4f".format(drive.requestedForward),
                "%.4f".format(drive.limitedForward),
                "%.4f".format(drive.requestedTurn),

                // ── Heading ────────────────────────────────────────────────────
                "%.4f".format(drive.heading),
                "%.4f".format(drive.targetHeading),
                "%.4f".format(drive.headingError),

                // ── IMU angular rates (all 3 axes) ─────────────────────────────
                "%.4f".format(drive.yawRate),
                "%.4f".format(drive.pitchRate),
                "%.4f".format(drive.rollRate),

                // ── Derived: heading angular acceleration ──────────────────────
                "%.4f".format(headingAccelDegS2),

                drive.headingHoldEnabled.i,

                // ── IMU orientation (all 3 angles) ─────────────────────────────
                "%.4f".format(drive.heading),       // yaw (duplicate for clarity)
                "%.4f".format(drive.pitchDegrees),
                "%.4f".format(drive.rollDegrees),

                // ── Heading PID corrections ────────────────────────────────────
                "%.6f".format(drive.proportionalCorrection),
                "%.6f".format(drive.integralCorrection),
                "%.6f".format(drive.derivativeCorrection),
                "%.6f".format(drive.headingCorrection),

                // ── Drive motor velocities (ticks/s) ───────────────────────────
                "%.2f".format(drive.leftTargetVelocity),
                "%.2f".format(drive.leftActualVelocity),
                "%.2f".format(drive.leftActualVelocity - drive.leftTargetVelocity),
                "%.2f".format(drive.rightTargetVelocity),
                "%.2f".format(drive.rightActualVelocity),
                "%.2f".format(drive.rightActualVelocity - drive.rightTargetVelocity),

                // ── Individual wheel speeds (mm/s) ─────────────────────────────
                "%.2f".format(drive.leftWheelSpeedMmPerSecond),
                "%.2f".format(drive.rightWheelSpeedMmPerSecond),

                // ── Drive motor encoder positions (ticks) ──────────────────────
                drive.leftEncoderPosition,
                drive.rightEncoderPosition,

                // ── Drive motor commanded duty cycle ───────────────────────────
                "%.4f".format(drive.leftMotorPower),
                "%.4f".format(drive.rightMotorPower),

                // ── Drive motor currents ────────────────────────────────────────
                "%.4f".format(drive.leftCurrentAmps),
                "%.4f".format(drive.rightCurrentAmps),
                "%.4f".format(drive.leftCurrentAmps + drive.rightCurrentAmps),

                // ── Robot summary ───────────────────────────────────────────────
                "%.2f".format(drive.linearSpeedMmPerSecond),
                "%.4f".format(drive.batteryVoltage),

                // ── Flywheel summary ────────────────────────────────────────────
                flywheel.enabled.i,
                flywheel.atSpeed.i,
                "%.2f".format(flywheel.targetRpm),
                "%.2f".format(flywheel.shaftRpm),
                "%.2f".format(flywheel.rpmDifference),
                "%.4f".format(flywheel.surfaceSpeedMetersPerSecond),
                "%.2f".format(flywheel.targetVelocity),

                // ── Flywheel primary motor ──────────────────────────────────────
                "%.2f".format(flywheel.primaryMotor.rpm),
                "%.2f".format(fwPrimaryRpmError),
                "%.2f".format(flywheel.primaryMotor.velocity),
                "%.4f".format(flywheel.primaryMotor.currentAmps),
                flywheel.primaryMotor.encoderPosition,
                "%.4f".format(flywheel.primaryMotor.motorPower),

                // ── Flywheel secondary motor ────────────────────────────────────
                "%.2f".format(flywheel.secondaryMotor.rpm),
                "%.2f".format(fwSecondaryRpmError),
                "%.2f".format(flywheel.secondaryMotor.velocity),
                "%.4f".format(flywheel.secondaryMotor.currentAmps),
                flywheel.secondaryMotor.encoderPosition,
                "%.4f".format(flywheel.secondaryMotor.motorPower),

                // ── Intake ──────────────────────────────────────────────────────
                intake.hexReversed.i,
                "%.4f".format(intake.motorPower),
                "%.4f".format(intake.hexMotorPower),
                "%.4f".format(intake.servo1Power),
                "%.4f".format(intake.servo2Power),
            ),
        )
    }

    fun stopLogging() {
        logger?.close()
        logger = null
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    private val Boolean.i: Int get() = if (this) 1 else 0

    private fun List<Double>.fmtOrNan(index: Int): String =
        if (index < size) "%.2f".format(get(index)) else "NaN"

    // ── CSV header — must match writeRow column order exactly ─────────────────

    private companion object {
        val HEADER = listOf(
            // Timing (4)
            "elapsed_sec", "loop_delta_ms",
            "absolute_timestamp_ms", "row_index",

            // Gamepad 1 axes (6)
            "gp1_left_stick_x", "gp1_left_stick_y",
            "gp1_right_stick_x", "gp1_right_stick_y",
            "gp1_left_trigger", "gp1_right_trigger",

            // Gamepad 1 buttons (14)
            "gp1_a", "gp1_b", "gp1_x", "gp1_y",
            "gp1_left_bumper", "gp1_right_bumper",
            "gp1_dpad_up", "gp1_dpad_down", "gp1_dpad_left", "gp1_dpad_right",
            "gp1_start", "gp1_back",
            "gp1_left_stick_btn", "gp1_right_stick_btn",

            // Gamepad 2 axes (6)
            "gp2_left_stick_x", "gp2_left_stick_y",
            "gp2_right_stick_x", "gp2_right_stick_y",
            "gp2_left_trigger", "gp2_right_trigger",

            // Gamepad 2 buttons (14)
            "gp2_a", "gp2_b", "gp2_x", "gp2_y",
            "gp2_left_bumper", "gp2_right_bumper",
            "gp2_dpad_up", "gp2_dpad_down", "gp2_dpad_left", "gp2_dpad_right",
            "gp2_start", "gp2_back",
            "gp2_left_stick_btn", "gp2_right_stick_btn",

            // REV Hub temperatures (2)
            "ctrl_hub_temp_c", "ext_hub_temp_c",

            // Drivetrain inputs (3)
            "drive_req_forward", "drive_lim_forward", "drive_req_turn",

            // Heading & IMU angular rates (7)
            "heading_deg", "target_heading_deg", "heading_error_deg",
            "yaw_rate_deg_s", "pitch_rate_deg_s", "roll_rate_deg_s",
            "heading_accel_deg_s2",

            // Heading hold flag (1)
            "heading_hold_enabled",

            // IMU full orientation angles (3)
            "yaw_deg", "pitch_deg", "roll_deg",

            // Heading PID (4)
            "pid_p", "pid_i", "pid_d", "pid_total",

            // Drive motor velocities + errors (6)
            "left_target_vel_ticks_s", "left_actual_vel_ticks_s", "left_vel_error_ticks_s",
            "right_target_vel_ticks_s", "right_actual_vel_ticks_s", "right_vel_error_ticks_s",

            // Individual wheel speeds (2)
            "left_wheel_speed_mm_s", "right_wheel_speed_mm_s",

            // Drive encoder positions (2)
            "left_encoder_pos_ticks", "right_encoder_pos_ticks",

            // Drive motor duty cycle (2)
            "left_motor_power", "right_motor_power",

            // Drive motor currents (3)
            "left_current_a", "right_current_a", "drive_total_current_a",

            // Robot summary (2)
            "linear_speed_mm_s", "battery_v",

            // Flywheel summary (7)
            "flywheel_enabled", "flywheel_at_speed",
            "flywheel_target_rpm", "flywheel_shaft_rpm", "flywheel_rpm_diff",
            "flywheel_surface_speed_m_s", "flywheel_target_vel_ticks_s",

            // Flywheel primary motor (6)
            "fw_primary_rpm", "fw_primary_rpm_error",
            "fw_primary_vel_ticks_s", "fw_primary_current_a",
            "fw_primary_enc_pos", "fw_primary_power",

            // Flywheel secondary motor (6)
            "fw_secondary_rpm", "fw_secondary_rpm_error",
            "fw_secondary_vel_ticks_s", "fw_secondary_current_a",
            "fw_secondary_enc_pos", "fw_secondary_power",

            // Intake (5)
            "intake_hex_enabled",
            "intake_motor_power", "intake_hex_motor_power",
            "intake_servo1_power", "intake_servo2_power",
        )
    }
}
