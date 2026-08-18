/*
package FGC.Vietnam

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.VoltageSensor
import FGC.Vietnam.Config.DrivetrainConfig
import FGC.Vietnam.DataLogger.TeleOpDatalogger
import com.turtletuner.TurtleTuner
import fgc.vietnam.robot01.Hardware.Drivetrain
import fgc.vietnam.robot01.Hardware.Flywheel
import fgc.vietnam.robot01.Hardware.Intake
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "🐢 TurtleTuner Bounded SysId", group = "Tuning")
class TurtleAutoTuneOpMode : OpMode() {

    object TunerConfig {
        @JvmField var MAX_FORWARD_METERS = 1.2
        @JvmField var MAX_BACKWARD_METERS = 1.2
        @JvmField var MAX_HEADING_DRIFT_DEG = 25.0
        @JvmField var RAMP_RATE_PWR_PER_SEC = 0.25
        @JvmField var STEP_POWER = 0.65
        @JvmField var STEP_DURATION_SEC = 1.5
    }

    enum class Routine {
        IDLE,
        QUASISTATIC_FORWARD,
        QUASISTATIC_BACKWARD,
        DYNAMIC_STEP_FORWARD,
        DYNAMIC_STEP_BACKWARD
    }

    private lateinit var drivetrain: Drivetrain
    private lateinit var flywheel: Flywheel
    private lateinit var intake: Intake
    private lateinit var voltageSensor: VoltageSensor
    private val datalogger = TeleOpDatalogger()

    private var activeRoutine = Routine.IDLE
    private var isRunning = false
    private var routineStartTimeMs = 0L
    private var commandedPower = 0.0

    // Watchdog initial references
    private var initLeftTicks = 0.0
    private var initRightTicks = 0.0
    private var initHeadingDeg = 0.0
    private var isBoundaryTripped = false
    private var tripReason: String? = null

    private var previousA = false
    private var previousB = false
    private var previousX = false
    private var previousY = false

    override fun init() {
        drivetrain = Drivetrain(hardwareMap)
        flywheel = Flywheel(hardwareMap)
        intake = Intake(hardwareMap)
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        hardwareMap.getAll(LynxModule::class.java).forEach { module ->
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        registerTurtleTunerConfig()

        telemetry.addLine("🐢 TurtleTuner Standalone Bounded Tuner Ready")
        telemetry.addLine("Controls: [A] Quasistatic Fwd | [B] Quasistatic Back | [X] Step Fwd | [Y] Step Back | [Dpad Down] Stop")
        telemetry.update()
    }

    override fun start() {
        if (!datalogger.isLogging) {
            datalogger.toggleLogging()
        }
    }

    override fun loop() {
        syncTurtleTunerConfig()
        val metersPerTick = DrivetrainConfig.millimetersPerEncoderTick / 1000.0

        val drive = drivetrain.drive(commandedPower, 0.0, false, voltageSensor.voltage)
        val flywheelState = flywheel.update(voltageSensor.voltage)

        val leftTicks = drive?.leftEncoderPosition?.toDouble() ?: 0.0
        val rightTicks = drive?.rightEncoderPosition?.toDouble() ?: 0.0
        val headingDeg = drive?.heading ?: 0.0

        // Consume commands from TurtleTuner's own robot-side data host.
        TurtleTuner.pollRemoteCommand()?.let { command ->
            if (command == "IDLE") {
                stopRoutine()
            } else {
                try {
                    startRoutine(Routine.valueOf(command), leftTicks, rightTicks, headingDeg)
                } catch (_: IllegalArgumentException) {
                    // The server validates routine names; keep this guard for version skew.
                }
            }
        }

        // Gamepad Triggers
        if (gamepad1.a && !previousA) startRoutine(Routine.QUASISTATIC_FORWARD, leftTicks, rightTicks, headingDeg)
        else if (gamepad1.b && !previousB) startRoutine(Routine.QUASISTATIC_BACKWARD, leftTicks, rightTicks, headingDeg)
        else if (gamepad1.x && !previousX) startRoutine(Routine.DYNAMIC_STEP_FORWARD, leftTicks, rightTicks, headingDeg)
        else if (gamepad1.y && !previousY) startRoutine(Routine.DYNAMIC_STEP_BACKWARD, leftTicks, rightTicks, headingDeg)
        else if (gamepad1.dpad_down) stopRoutine()

        previousA = gamepad1.a
        previousB = gamepad1.b
        previousX = gamepad1.x
        previousY = gamepad1.y

        // Execute Routine & Enforce Boundary Watchdog
        if (isRunning) {
            val leftM = (leftTicks - initLeftTicks) * metersPerTick
            val rightM = (rightTicks - initRightTicks) * metersPerTick
            val avgMeters = (leftM + rightM) / 2.0

            var headingErr = abs(headingDeg - initHeadingDeg)
            while (headingErr > 180.0) headingErr = abs(headingErr - 360.0)

            if (avgMeters > TunerConfig.MAX_FORWARD_METERS) {
                tripWatchdog("Forward limit exceeded: %.2fm > %.2fm".format(avgMeters, TunerConfig.MAX_FORWARD_METERS))
            } else if (avgMeters < -TunerConfig.MAX_BACKWARD_METERS) {
                tripWatchdog("Backward limit exceeded: %.2fm < -%.2fm".format(avgMeters, TunerConfig.MAX_BACKWARD_METERS))
            } else if (headingErr > TunerConfig.MAX_HEADING_DRIFT_DEG) {
                tripWatchdog("Heading drift exceeded: %.1f° > %.1f°".format(headingErr, TunerConfig.MAX_HEADING_DRIFT_DEG))
            } else {
                val elapsedSec = (System.currentTimeMillis() - routineStartTimeMs) / 1000.0
                when (activeRoutine) {
                    Routine.QUASISTATIC_FORWARD -> {
                        commandedPower = min(1.0, elapsedSec * TunerConfig.RAMP_RATE_PWR_PER_SEC)
                        if (commandedPower >= 1.0 && elapsedSec >= 5.0) stopRoutine()
                    }
                    Routine.QUASISTATIC_BACKWARD -> {
                        commandedPower = max(-1.0, -elapsedSec * TunerConfig.RAMP_RATE_PWR_PER_SEC)
                        if (commandedPower <= -1.0 && elapsedSec >= 5.0) stopRoutine()
                    }
                    Routine.DYNAMIC_STEP_FORWARD -> {
                        commandedPower = TunerConfig.STEP_POWER
                        if (elapsedSec >= TunerConfig.STEP_DURATION_SEC) stopRoutine()
                    }
                    Routine.DYNAMIC_STEP_BACKWARD -> {
                        commandedPower = -TunerConfig.STEP_POWER
                        if (elapsedSec >= TunerConfig.STEP_DURATION_SEC) stopRoutine()
                    }
                    Routine.IDLE -> commandedPower = 0.0
                }
            }
        } else {
            commandedPower = 0.0
        }

        // Log to CSV
        if (datalogger.isLogging && drive != null) {
            datalogger.writeRow(gamepad1, gamepad2, drive, flywheelState, intake, emptyList())
        }

        // Telemetry is hosted by TurtleTuner, independently of FTC Dashboard.
        TurtleTuner.publishTelemetry(
            mapOf(
                "TT_Routine" to activeRoutine.name,
                "TT_Power" to commandedPower,
                "TT_Watchdog_Tripped" to isBoundaryTripped,
                "TT_Trip_Reason" to (tripReason ?: ""),
                "Left Ticks" to leftTicks,
                "Right Ticks" to rightTicks,
                "Heading Deg" to headingDeg,
                "Battery (V)" to voltageSensor.voltage
            )
        )

        telemetry.addData("Routine", activeRoutine.name)
        telemetry.addData("Commanded Power", "%.3f", commandedPower)
        telemetry.addData("Watchdog Safe", !isBoundaryTripped)
        if (isBoundaryTripped) telemetry.addData("⚠️ Trip Reason", tripReason)
        telemetry.update()
    }

    private fun startRoutine(routine: Routine, leftTicks: Double, rightTicks: Double, headingDeg: Double) {
        activeRoutine = routine
        isRunning = true
        routineStartTimeMs = System.currentTimeMillis()
        initLeftTicks = leftTicks
        initRightTicks = rightTicks
        initHeadingDeg = headingDeg
        isBoundaryTripped = false
        tripReason = null
        commandedPower = 0.0
    }

    private fun tripWatchdog(reason: String) {
        isRunning = false
        activeRoutine = Routine.IDLE
        commandedPower = 0.0
        isBoundaryTripped = true
        tripReason = reason
    }

    private fun stopRoutine() {
        isRunning = false
        activeRoutine = Routine.IDLE
        commandedPower = 0.0
    }

    private fun registerTurtleTunerConfig() {
        TurtleTuner.registerConfig(
            "TunerConfig",
            mapOf(
                "MAX_FORWARD_METERS" to TunerConfig.MAX_FORWARD_METERS,
                "MAX_BACKWARD_METERS" to TunerConfig.MAX_BACKWARD_METERS,
                "MAX_HEADING_DRIFT_DEG" to TunerConfig.MAX_HEADING_DRIFT_DEG,
                "RAMP_RATE_PWR_PER_SEC" to TunerConfig.RAMP_RATE_PWR_PER_SEC,
                "STEP_POWER" to TunerConfig.STEP_POWER,
                "STEP_DURATION_SEC" to TunerConfig.STEP_DURATION_SEC
            )
        )
        TurtleTuner.registerConfig(
            "DrivetrainConfig",
            mapOf(
                "ACTIVE_HEADING_KP" to DrivetrainConfig.ACTIVE_HEADING_KP,
                "ACTIVE_HEADING_KI" to DrivetrainConfig.ACTIVE_HEADING_KI,
                "ACTIVE_HEADING_KD" to DrivetrainConfig.ACTIVE_HEADING_KD,
                "ACTIVE_HEADING_KS" to DrivetrainConfig.ACTIVE_HEADING_KS,
                "ACTIVE_HEADING_HOLD_DEADBAND_DEG" to DrivetrainConfig.ACTIVE_HEADING_HOLD_DEADBAND_DEG,
                "ACTIVE_HEADING_TURN_DEADBAND" to DrivetrainConfig.ACTIVE_HEADING_TURN_DEADBAND,
                "DRIVE_SPEED_MULTIPLIER" to DrivetrainConfig.DRIVE_SPEED_MULTIPLIER,
                "ENABLE_ACTIVE_HEADING_CORRECTION" to DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION,
                "DATALOG_ENABLED" to DrivetrainConfig.DATALOG_ENABLED
            )
        )
    }

    private fun syncTurtleTunerConfig() {
        TunerConfig.MAX_FORWARD_METERS = TurtleTuner.configDouble(
            "TunerConfig", "MAX_FORWARD_METERS", TunerConfig.MAX_FORWARD_METERS
        )
        TunerConfig.MAX_BACKWARD_METERS = TurtleTuner.configDouble(
            "TunerConfig", "MAX_BACKWARD_METERS", TunerConfig.MAX_BACKWARD_METERS
        )
        TunerConfig.MAX_HEADING_DRIFT_DEG = TurtleTuner.configDouble(
            "TunerConfig", "MAX_HEADING_DRIFT_DEG", TunerConfig.MAX_HEADING_DRIFT_DEG
        )
        TunerConfig.RAMP_RATE_PWR_PER_SEC = TurtleTuner.configDouble(
            "TunerConfig", "RAMP_RATE_PWR_PER_SEC", TunerConfig.RAMP_RATE_PWR_PER_SEC
        )
        TunerConfig.STEP_POWER = TurtleTuner.configDouble(
            "TunerConfig", "STEP_POWER", TunerConfig.STEP_POWER
        )
        TunerConfig.STEP_DURATION_SEC = TurtleTuner.configDouble(
            "TunerConfig", "STEP_DURATION_SEC", TunerConfig.STEP_DURATION_SEC
        )

        DrivetrainConfig.ACTIVE_HEADING_KP = TurtleTuner.configDouble(
            "DrivetrainConfig", "ACTIVE_HEADING_KP", DrivetrainConfig.ACTIVE_HEADING_KP
        )
        DrivetrainConfig.ACTIVE_HEADING_KI = TurtleTuner.configDouble(
            "DrivetrainConfig", "ACTIVE_HEADING_KI", DrivetrainConfig.ACTIVE_HEADING_KI
        )
        DrivetrainConfig.ACTIVE_HEADING_KD = TurtleTuner.configDouble(
            "DrivetrainConfig", "ACTIVE_HEADING_KD", DrivetrainConfig.ACTIVE_HEADING_KD
        )
        DrivetrainConfig.ACTIVE_HEADING_KS = TurtleTuner.configDouble(
            "DrivetrainConfig", "ACTIVE_HEADING_KS", DrivetrainConfig.ACTIVE_HEADING_KS
        )
        DrivetrainConfig.ACTIVE_HEADING_HOLD_DEADBAND_DEG = TurtleTuner.configDouble(
            "DrivetrainConfig",
            "ACTIVE_HEADING_HOLD_DEADBAND_DEG",
            DrivetrainConfig.ACTIVE_HEADING_HOLD_DEADBAND_DEG
        )
        DrivetrainConfig.ACTIVE_HEADING_TURN_DEADBAND = TurtleTuner.configDouble(
            "DrivetrainConfig",
            "ACTIVE_HEADING_TURN_DEADBAND",
            DrivetrainConfig.ACTIVE_HEADING_TURN_DEADBAND
        )
        DrivetrainConfig.DRIVE_SPEED_MULTIPLIER = TurtleTuner.configDouble(
            "DrivetrainConfig", "DRIVE_SPEED_MULTIPLIER", DrivetrainConfig.DRIVE_SPEED_MULTIPLIER
        )
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION = TurtleTuner.configBoolean(
            "DrivetrainConfig",
            "ENABLE_ACTIVE_HEADING_CORRECTION",
            DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION
        )
        DrivetrainConfig.DATALOG_ENABLED = TurtleTuner.configBoolean(
            "DrivetrainConfig", "DATALOG_ENABLED", DrivetrainConfig.DATALOG_ENABLED
        )
    }

    override fun stop() {
        stopRoutine()
        drivetrain.stop()
        flywheel.stop()
        intake.stopMotor()
        datalogger.stopLogging()
    }
}
*/
