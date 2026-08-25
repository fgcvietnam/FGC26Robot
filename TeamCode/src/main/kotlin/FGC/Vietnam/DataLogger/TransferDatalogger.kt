package FGC.Vietnam.DataLogger

import com.qualcomm.robotcore.hardware.Gamepad
import fgc.vietnam.robot01.Hardware.FlywheelTelemetry
import fgc.vietnam.robot01.Hardware.IntakeTelemetry

/**
 * Dedicated CSV Datalogger for Transfer & Shooting Mechanism Testing during TeleOp.
 * Strictly focused on:
 * - Intake Roller & Extension Servos
 * - CoreHex Transfer Mechanism
 * - Dual Shooter / Flywheel Motors
 * - Electrical Power, Battery Voltage, and Driver Inputs
 */
internal class TransferDatalogger {

    private var logger: Datalogger? = null
    private var sessionStartNanos: Long = 0L
    private var lastRowNanos: Long = 0L
    private var rowIndex: Int = 0

    val isLogging: Boolean get() = logger != null
    val rowCount: Int get() = logger?.rowCount ?: 0
    val fileName: String? get() = logger?.file?.name
    val filePath: String? get() = logger?.file?.absolutePath
    val errorMessage: String? get() = logger?.errorMessage

    fun startLogging(prefix: String = "transfer_test"): Boolean {
        if (logger == null) {
            val now = System.nanoTime()
            sessionStartNanos = now
            lastRowNanos = now
            rowIndex = 0
            logger = Datalogger.create(prefix = prefix, header = HEADER)
            return true
        }
        return false
    }

    /** Toggle start / stop. Returns true if logging just started. */
    fun toggleLogging(prefix: String = "transfer_test"): Boolean {
        return if (logger == null) {
            startLogging(prefix)
        } else {
            stopLogging()
            false
        }
    }

    fun stopLogging() {
        logger?.close()
        logger = null
    }

    fun writeRow(
        intakeTelemetry: IntakeTelemetry?,
        flywheelTelemetry: FlywheelTelemetry?,
        batteryVoltage: Double,
        isTransferring: Boolean,
        isOuttaking: Boolean,
        gamepad1: Gamepad,
        gamepad2: Gamepad? = null,
        hubTemperaturesCelsius: List<Double> = emptyList()
    ) {
        val log = logger ?: return

        val nowNanos = System.nanoTime()
        val elapsedSec = (nowNanos - sessionStartNanos) / 1_000_000_000.0
        val deltaMs = (nowNanos - lastRowNanos) / 1_000_000.0
        val frequencyHz = if (deltaMs > 0.0) 1000.0 / deltaMs else 0.0
        lastRowNanos = nowNanos
        rowIndex++

        val ctrlHubTemp = if (hubTemperaturesCelsius.isNotEmpty()) hubTemperaturesCelsius[0] else Double.NaN
        val extHubTemp = if (hubTemperaturesCelsius.size > 1) hubTemperaturesCelsius[1] else Double.NaN

        // Intake metrics
        val intakeState = intakeTelemetry?.state?.name ?: "OFF"
        val intakePwr = intakeTelemetry?.motorPower ?: 0.0
        val intakeCurr = intakeTelemetry?.motorCurrentAmps ?: 0.0
        val intakeWatts = intakeCurr * batteryVoltage * kotlin.math.abs(intakePwr)
        val intakeJam = if (intakeTelemetry?.intakeJamConfirmed == true) 1 else 0
        val leftSlidePos = intakeTelemetry?.leftSlidePosition?.name ?: "UNKNOWN"
        val rightSlidePos = intakeTelemetry?.rightSlidePosition?.name ?: "UNKNOWN"
        val sLeftBelow = intakeTelemetry?.servoLeftBelowPower ?: 0.0
        val sRightBelow = intakeTelemetry?.servoRightBelowPower ?: 0.0
        val sLeftAbove = intakeTelemetry?.servoLeftAbovePower ?: 0.0
        val sRightAbove = intakeTelemetry?.servoRightAbovePower ?: 0.0
        val lLimit = if (intakeTelemetry?.leftLimitSwitchPressed == true) 1 else 0
        val rLimit = if (intakeTelemetry?.rightLimitSwitchPressed == true) 1 else 0
        val lMag = if (intakeTelemetry?.leftMagneticSwitchPressed == true) 1 else 0
        val rMag = if (intakeTelemetry?.rightMagneticSwitchPressed == true) 1 else 0

        // Transfer CoreHex metrics
        val hexPwr = intakeTelemetry?.hexMotorPower ?: 0.0
        val hexCurr = intakeTelemetry?.hexMotorCurrentAmps ?: 0.0
        val hexWatts = hexCurr * batteryVoltage * kotlin.math.abs(hexPwr)
        val hexJam = if (intakeTelemetry?.transferJamConfirmed == true) 1 else 0
        val isUnjamming = if (intakeTelemetry?.isUnjamming == true) 1 else 0

        // Flywheel Shooter metrics
        val shooterEnabled = if (flywheelTelemetry?.enabled == true) 1 else 0
        val shooterAtSpeed = if (flywheelTelemetry?.atSpeed == true) 1 else 0
        val targetRpm = flywheelTelemetry?.targetRpm ?: 0.0

        val leftShooter = flywheelTelemetry?.leftShooterMotor
        val leftPwr = leftShooter?.motorPower ?: 0.0
        val leftCurr = leftShooter?.currentAmps ?: 0.0
        val leftWatts = leftCurr * batteryVoltage * kotlin.math.abs(leftPwr)
        val leftRpm = leftShooter?.rpm ?: 0.0
        val leftVel = leftShooter?.velocity ?: 0.0
        val leftPos = leftShooter?.encoderPosition ?: 0

        val rightShooter = flywheelTelemetry?.rightShooterMotor
        val rightPwr = rightShooter?.motorPower ?: 0.0
        val rightCurr = rightShooter?.currentAmps ?: 0.0
        val rightWatts = rightCurr * batteryVoltage * kotlin.math.abs(rightPwr)
        val rightRpm = rightShooter?.rpm ?: 0.0
        val rightVel = rightShooter?.velocity ?: 0.0
        val rightPos = rightShooter?.encoderPosition ?: 0

        val rpmDiff = flywheelTelemetry?.rpmDifference ?: 0.0
        val shaftRpm = flywheelTelemetry?.shaftRpm ?: 0.0
        val surfaceSpeed = flywheelTelemetry?.surfaceSpeedMetersPerSecond ?: 0.0

        // Total mechanism electrical draw (Intake + Transfer + 2 Shooter Motors)
        val totalMechCurrent = intakeCurr + hexCurr + leftCurr + rightCurr
        val totalMechWatts = intakeWatts + hexWatts + leftWatts + rightWatts

        log.writeRow(
            listOf(
                // ── Timing ──
                "%.4f".format(elapsedSec),
                "%.3f".format(deltaMs),
                "%.1f".format(frequencyHz),
                System.currentTimeMillis(),
                rowIndex,

                // ── Battery & System ──
                "%.3f".format(batteryVoltage),
                if (ctrlHubTemp.isNaN()) "" else "%.1f".format(ctrlHubTemp),
                if (extHubTemp.isNaN()) "" else "%.1f".format(extHubTemp),

                // ── Driver Commands ──
                if (isTransferring) 1 else 0,
                if (isOuttaking) 1 else 0,
                intakeState,
                shooterEnabled,
                if (gamepad1.right_bumper || (gamepad2?.right_bumper == true)) 1 else 0,
                if (gamepad1.left_bumper || (gamepad2?.left_bumper == true)) 1 else 0,
                if (gamepad1.square || (gamepad2?.square == true)) 1 else 0,
                if (gamepad1.circle || (gamepad2?.circle == true)) 1 else 0,

                // ── Intake Roller & Extension ──
                intakeState,
                "%.3f".format(intakePwr),
                "%.3f".format(intakeCurr),
                "%.2f".format(intakeWatts),
                intakeJam,
                leftSlidePos,
                rightSlidePos,
                "%.3f".format(sLeftBelow),
                "%.3f".format(sRightBelow),
                "%.3f".format(sLeftAbove),
                "%.3f".format(sRightAbove),
                lLimit,
                rLimit,
                lMag,
                rMag,

                // ── CoreHex Transfer ──
                "%.3f".format(hexPwr),
                "%.3f".format(hexCurr),
                "%.2f".format(hexWatts),
                hexJam,
                isUnjamming,

                // ── Shooter / Flywheel (2 Motors) ──
                shooterEnabled,
                shooterAtSpeed,
                "%.1f".format(targetRpm),

                "%.3f".format(leftPwr),
                "%.3f".format(leftCurr),
                "%.2f".format(leftWatts),
                "%.1f".format(leftRpm),
                "%.1f".format(leftVel),
                leftPos,

                "%.3f".format(rightPwr),
                "%.3f".format(rightCurr),
                "%.2f".format(rightWatts),
                "%.1f".format(rightRpm),
                "%.1f".format(rightVel),
                rightPos,

                "%.1f".format(rpmDiff),
                "%.1f".format(shaftRpm),
                "%.2f".format(surfaceSpeed),

                // ── Total Subsystem Electrical ──
                "%.3f".format(totalMechCurrent),
                "%.2f".format(totalMechWatts)
            )
        )
    }

    companion object {
        val HEADER = listOf(
            // Timing
            "elapsed_sec",
            "loop_dt_ms",
            "loop_hz",
            "absolute_timestamp_ms",
            "sample_index",

            // Battery & System
            "battery_voltage_v",
            "ctrl_hub_temp_c",
            "ext_hub_temp_c",

            // Driver Commands
            "is_transferring",
            "is_outtaking",
            "intake_state_cmd",
            "flywheel_enabled_cmd",
            "gp1_rb_transfer",
            "gp1_lb_outtake",
            "gp1_square_intake",
            "gp1_circle_flywheel",

            // Intake Roller & Extension
            "intake_state",
            "intake_roller_power",
            "intake_roller_current_a",
            "intake_roller_power_w",
            "intake_jam_confirmed",
            "left_slide_pos",
            "right_slide_pos",
            "servo_left_below_pwr",
            "servo_right_below_pwr",
            "servo_left_above_pwr",
            "servo_right_above_pwr",
            "left_limit_pressed",
            "right_limit_pressed",
            "left_mag_pressed",
            "right_mag_pressed",

            // CoreHex Transfer
            "transfer_hex_power",
            "transfer_hex_current_a",
            "transfer_hex_power_w",
            "transfer_jam_confirmed",
            "transfer_is_unjamming",

            // Shooter / Flywheel (2 Motors)
            "shooter_enabled",
            "shooter_at_speed",
            "shooter_target_rpm",

            "left_shooter_power",
            "left_shooter_current_a",
            "left_shooter_power_w",
            "left_shooter_rpm",
            "left_shooter_velocity_ticks_s",
            "left_shooter_encoder_ticks",

            "right_shooter_power",
            "right_shooter_current_a",
            "right_shooter_power_w",
            "right_shooter_rpm",
            "right_shooter_velocity_ticks_s",
            "right_shooter_encoder_ticks",

            "shooter_rpm_diff",
            "shooter_shaft_rpm",
            "shooter_surface_speed_m_s",

            // Total Mechanism Electrical
            "total_mechanism_current_a",
            "total_mechanism_power_w"
        )
    }
}
