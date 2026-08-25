package FGC.Vietnam.DataLogger

import com.qualcomm.robotcore.hardware.Gamepad
import fgc.vietnam.robot01.Hardware.ClimbTelemetry
import kotlin.math.abs
import kotlin.math.sqrt

/**
 * Dedicated high-granularity CSV datalogger for the Climb mechanism during TeleOp.
 * Logs full dual-motor electrical/mechanical specs, 3D IMU orientation & rates,
 * distance sensor, driver controls, and Hub metrics.
 */
internal class ClimbDatalogger {

    private var logger: Datalogger? = null
    private var sessionStartNanos: Long = 0L
    private var lastRowNanos: Long = 0L
    private var previousYawRate: Double = 0.0
    private var previousPitchRate: Double = 0.0
    private var previousRollRate: Double = 0.0
    private var rowIndex: Int = 0

    val isLogging: Boolean get() = logger != null
    val rowCount: Int get() = logger?.rowCount ?: 0
    val fileName: String? get() = logger?.file?.name
    val filePath: String? get() = logger?.file?.absolutePath
    val errorMessage: String? get() = logger?.errorMessage

    fun startLogging(prefix: String = "climb_teleop"): Boolean {
        if (logger == null) {
            val now = System.nanoTime()
            sessionStartNanos = now
            lastRowNanos = now
            previousYawRate = 0.0
            previousPitchRate = 0.0
            previousRollRate = 0.0
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
        climb: ClimbTelemetry,
        batteryVoltage: Double,
        yawDeg: Double,
        pitchDeg: Double,
        rollDeg: Double,
        yawRateDegS: Double,
        pitchRateDegS: Double,
        rollRateDegS: Double,
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

        val dtSec = if (deltaMs > 0.0) deltaMs / 1000.0 else 0.001

        // Derived 3D angular accelerations (deg/s^2)
        val yawAccelDegS2 = (yawRateDegS - previousYawRate) / dtSec
        val pitchAccelDegS2 = (pitchRateDegS - previousPitchRate) / dtSec
        val rollAccelDegS2 = (rollRateDegS - previousRollRate) / dtSec

        previousYawRate = yawRateDegS
        previousPitchRate = pitchRateDegS
        previousRollRate = rollRateDegS

        val totalTiltDeg = sqrt(pitchDeg * pitchDeg + rollDeg * rollDeg)
        val totalAngularRate = sqrt(yawRateDegS * yawRateDegS + pitchRateDegS * pitchRateDegS + rollRateDegS * rollRateDegS)

        val ctrlHubTemp = if (hubTemperaturesCelsius.isNotEmpty()) hubTemperaturesCelsius[0] else Double.NaN
        val extHubTemp = if (hubTemperaturesCelsius.size > 1) hubTemperaturesCelsius[1] else Double.NaN

        log.writeRow(
            listOf(
                // ── Timing ──
                "%.4f".format(elapsedSec),
                "%.3f".format(deltaMs),
                "%.1f".format(frequencyHz),
                System.currentTimeMillis(),
                rowIndex,

                // ── Climb State & Driver Inputs ──
                climb.climbState,
                "%.4f".format(gamepad1.right_trigger),
                "%.4f".format(gamepad1.left_trigger),
                if (gamepad1.triangle || (gamepad2?.triangle == true)) 1 else 0,
                if (gamepad1.touchpad || (gamepad2?.touchpad == true)) 1 else 0,
                if (climb.shouldHold) 1 else 0,
                "%.4f".format(climb.holdPower),

                // ── Motor Above (All Specs) ──
                "%.4f".format(climb.motorAbovePower),
                climb.motorAbovePositionTicks,
                "%.2f".format(climb.motorAboveVelocityTicksPerSec),
                "%.2f".format(climb.motorAboveRpm),
                "%.2f".format(climb.shaftAboveRpm),
                "%.4f".format(climb.motorAboveCurrentAmps),
                "%.4f".format(climb.motorAbovePowerWatts),

                // ── Motor Below (All Specs) ──
                "%.4f".format(climb.motorBelowPower),
                climb.motorBelowPositionTicks,
                "%.2f".format(climb.motorBelowVelocityTicksPerSec),
                "%.2f".format(climb.motorBelowRpm),
                "%.2f".format(climb.shaftBelowRpm),
                "%.4f".format(climb.motorBelowCurrentAmps),
                "%.4f".format(climb.motorBelowPowerWatts),

                // ── Dual Motor Differentials & Totals ──
                climb.encoderPositionDiffTicks,
                "%.2f".format(climb.motorVelocityDiffTicksPerSec),
                "%.4f".format(climb.totalCurrentAmps),
                "%.4f".format(climb.currentDiffAmps),
                "%.4f".format(climb.totalPowerWatts),

                // ── Aux Actuators & Distance ──
                "%.4f".format(climb.servoPower),
                if (climb.distanceMm.isNaN()) "NaN" else "%.2f".format(climb.distanceMm),

                // ── System & Hubs ──
                "%.3f".format(batteryVoltage),
                if (ctrlHubTemp.isNaN()) "NaN" else "%.1f".format(ctrlHubTemp),
                if (extHubTemp.isNaN()) "NaN" else "%.1f".format(extHubTemp),

                // ── IMU in All Directions (3D Orientation & Rates) ──
                "%.2f".format(yawDeg),
                "%.2f".format(pitchDeg),
                "%.2f".format(rollDeg),
                "%.2f".format(totalTiltDeg),
                "%.2f".format(yawRateDegS),
                "%.2f".format(pitchRateDegS),
                "%.2f".format(rollRateDegS),
                "%.2f".format(totalAngularRate),
                "%.2f".format(yawAccelDegS2),
                "%.2f".format(pitchAccelDegS2),
                "%.2f".format(rollAccelDegS2),

                // ── Gamepad 1 Axes ──
                "%.4f".format(gamepad1.left_stick_x),
                "%.4f".format(gamepad1.left_stick_y),
                "%.4f".format(gamepad1.right_stick_x),
                "%.4f".format(gamepad1.right_stick_y),

                // ── Gamepad 1 Buttons ──
                if (gamepad1.a) 1 else 0,
                if (gamepad1.b) 1 else 0,
                if (gamepad1.x) 1 else 0,
                if (gamepad1.y) 1 else 0,
                if (gamepad1.left_bumper) 1 else 0,
                if (gamepad1.right_bumper) 1 else 0,
                if (gamepad1.dpad_up) 1 else 0,
                if (gamepad1.dpad_down) 1 else 0,
                if (gamepad1.dpad_left) 1 else 0,
                if (gamepad1.dpad_right) 1 else 0
            )
        )
    }

    companion object {
        val HEADER = listOf(
            "elapsed_sec",
            "loop_dt_ms",
            "loop_hz",
            "absolute_timestamp_ms",
            "sample_index",
            "climb_state",
            "driver_climb_forward_input",
            "driver_climb_hold_input",
            "driver_climb_extend_btn",
            "driver_climb_retract_btn",
            "should_hold",
            "hold_power",
            "motor_above_power",
            "motor_above_position_ticks",
            "motor_above_vel_ticks_s",
            "motor_above_motor_rpm",
            "motor_above_shaft_rpm",
            "motor_above_current_a",
            "motor_above_power_w",
            "motor_below_power",
            "motor_below_position_ticks",
            "motor_below_vel_ticks_s",
            "motor_below_motor_rpm",
            "motor_below_shaft_rpm",
            "motor_below_current_a",
            "motor_below_power_w",
            "encoder_diff_ticks",
            "velocity_diff_ticks_s",
            "total_climb_current_a",
            "current_diff_a",
            "total_climb_power_w",
            "servo_power",
            "distance_sensor_mm",
            "battery_voltage_v",
            "ctrl_hub_temp_c",
            "ext_hub_temp_c",
            "imu_yaw_deg",
            "imu_pitch_deg",
            "imu_roll_deg",
            "total_tilt_deg",
            "yaw_rate_deg_s",
            "pitch_rate_deg_s",
            "roll_rate_deg_s",
            "total_angular_rate_deg_s",
            "yaw_accel_deg_s2",
            "pitch_accel_deg_s2",
            "roll_accel_deg_s2",
            "gp1_left_stick_x",
            "gp1_left_stick_y",
            "gp1_right_stick_x",
            "gp1_right_stick_y",
            "gp1_a",
            "gp1_b",
            "gp1_x",
            "gp1_y",
            "gp1_lb",
            "gp1_rb",
            "gp1_dpad_up",
            "gp1_dpad_down",
            "gp1_dpad_left",
            "gp1_dpad_right"
        )
    }
}
