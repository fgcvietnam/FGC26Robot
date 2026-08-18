package FGC.Vietnam

import RoadRunner.Drawing.drawRobot
import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Blinker
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import FGC.Vietnam.Config.DrivetrainConfig
import FGC.Vietnam.Config.RobotConfig
import FGC.Vietnam.DataLogger.DrivetrainDatalogger
import fgc.vietnam.robot01.Hardware.Drivetrain
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sign

@Config
object DrivetrainFigureEightConfig {
    @JvmField var AUTO_START_ON_PLAY = true
    @JvmField var SEGMENT_DISTANCE_METERS = 1.5
    @JvmField var TOTAL_CYCLES = 10 // 10 cycles of 16 steps = 160 steps total

    // Drive power parameters
    @JvmField var MAX_DRIVE_POWER = 0.70
    @JvmField var MIN_DRIVE_POWER = 0.35
    @JvmField var RAMP_UP_DISTANCE_METERS = 0.30
    @JvmField var RAMP_DOWN_DISTANCE_METERS = 0.45
    @JvmField var DRIVE_SETTLE_SECONDS = 0.20

    // Turn limits & settle criteria (PID gains directly use DrivetrainConfig)
    @JvmField var MAX_TURN_POWER = 0.50
    @JvmField var MIN_TURN_POWER = 0.10
    @JvmField var TURN_TOLERANCE_DEG = 1.0
    @JvmField var TURN_MAX_VELOCITY_DEG_S = 3.0
    @JvmField var TURN_SETTLE_SECONDS = 0.15

    @JvmField var STEP_TIMEOUT_SECONDS = 12.0
    // Matches TeleOp: FTC stick-forward is negative, and negative drive power is physical forward.
    @JvmField var FORWARD_DIRECTION_SIGN = -1.0
    @JvmField var ENABLE_MANUAL_DRIVE_WHEN_IDLE = true
}

/**
 * Automated Drivetrain Figure-8 / Double-Box Loop Datalogging Autonomous OpMode.
 *
 * Sequence per cycle (16 sub-steps):
 * 1. Fwd 1.5m (0 deg) -> 2. Turn Right 90 deg (-90 deg)
 * 3. Fwd 1.5m (-90 deg) -> 4. Turn Right 90 deg (180 deg)
 * 5. Fwd 1.5m (180 deg) -> 6. Turn Right 90 deg (90 deg)
 * 7. Fwd 1.5m (90 deg) -> 8. Turn Right 90 deg (0 deg - Box 1 Complete)
 * 9. Fwd 1.5m (0 deg) -> 10. Turn Left 90 deg (90 deg)
 * 11. Fwd 1.5m (90 deg) -> 12. Turn Left 90 deg (180 deg)
 * 13. Fwd 1.5m (180 deg) -> 14. Turn Left 90 deg (-90 deg)
 * 15. Fwd 1.5m (-90 deg) -> 16. Turn Left 90 deg (0 deg - Box 2 Complete, Return to Start)
 *
 * Repeats 10 times (160 steps total = 120m driving + 80 turns).
 * Full high-detail datalogging with extensive heading tuning telemetry.
 */
@Autonomous(name = "Drivetrain Auto Datalog (Figure 8 Loops)", group = "Autonomous")
class DrivetrainFigureEightDatalogOpMode : OpMode() {

    enum class TestPhase {
        IDLE,
        DRIVING,
        DRIVE_SETTLING,
        TURNING,
        TURN_SETTLING,
        COMPLETED,
        ABORTED
    }

    data class StepDef(
        val stepNumber: Int,
        val isTurn: Boolean,
        val targetDistanceM: Double,
        val targetHeadingDeg: Double,
        val description: String
    )

    private val cycleSteps = listOf(
        StepDef(1,  isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = 0.0,    description = "Fwd 1.5m (0 deg)"),
        StepDef(2,  isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = -90.0,  description = "Turn Right 90 deg (-90 deg)"),
        StepDef(3,  isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = -90.0,  description = "Fwd 1.5m (-90 deg)"),
        StepDef(4,  isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = 180.0,  description = "Turn Right 90 deg (180 deg)"),
        StepDef(5,  isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = 180.0,  description = "Fwd 1.5m (180 deg)"),
        StepDef(6,  isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = 90.0,   description = "Turn Right 90 deg (90 deg)"),
        StepDef(7,  isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = 90.0,   description = "Fwd 1.5m (90 deg)"),
        StepDef(8,  isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = 0.0,    description = "Turn Right 90 deg (0 deg - Return to Origin)"),
        StepDef(9,  isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = 0.0,    description = "Fwd 1.5m (0 deg)"),
        StepDef(10, isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = 90.0,   description = "Turn Left 90 deg (90 deg)"),
        StepDef(11, isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = 90.0,   description = "Fwd 1.5m (90 deg)"),
        StepDef(12, isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = 180.0,  description = "Turn Left 90 deg (180 deg)"),
        StepDef(13, isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = 180.0,  description = "Fwd 1.5m (180 deg)"),
        StepDef(14, isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = -90.0,  description = "Turn Left 90 deg (-90 deg)"),
        StepDef(15, isTurn = false, targetDistanceM = 1.5, targetHeadingDeg = -90.0,  description = "Fwd 1.5m (-90 deg)"),
        StepDef(16, isTurn = true,  targetDistanceM = 0.0, targetHeadingDeg = 0.0,    description = "Turn Left 90 deg (0 deg - Return to Origin)")
    )

    private lateinit var drivetrain: Drivetrain
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var dashboard: FtcDashboard

    private val datalogger = DrivetrainDatalogger()

    private var phase = TestPhase.IDLE
    private var currentCycle = 0 // 1..TOTAL_CYCLES
    private var currentStepInCycle = 0 // 0..15
    private var globalStepIndex = 0 // 1..(TOTAL_CYCLES * 16)
    private var cumulativeDistanceMeters = 0.0

    private var segmentStartLeftTicks = 0
    private var segmentStartRightTicks = 0
    private var segmentProgressMeters = 0.0
    private var segmentRemainingMeters = 0.0

    // Turn PID state
    private var turnIntegral = 0.0
    private var lastTurnErrorRad = 0.0
    private var lastTurnOutput = 0.0

    private val stepTimer = ElapsedTime()
    private val totalTestTimer = ElapsedTime()
    private val settleTimer = ElapsedTime()
    private val loopTimer = ElapsedTime()

    private var lastTempQueryTimeMs = 0L
    private var cachedHubTemps: List<Double> = emptyList()

    private var previousA = false
    private var previousB = false
    private var abortReason: String? = null
    private var savedFileName: String? = null
    private var savedRowCount: Int = 0

    private val runningPattern = listOf(
        Blinker.Step(Color.BLUE, 300, TimeUnit.MILLISECONDS),
        Blinker.Step(Color.CYAN, 300, TimeUnit.MILLISECONDS)
    )

    private val completePattern = listOf(
        Blinker.Step(Color.GREEN, 1, TimeUnit.SECONDS)
    )

    private val abortPattern = listOf(
        Blinker.Step(Color.RED, 200, TimeUnit.MILLISECONDS),
        Blinker.Step(Color.BLACK, 200, TimeUnit.MILLISECONDS)
    )

    override fun init() {
        dashboard = FtcDashboard.getInstance()
        drivetrain = Drivetrain(hardwareMap)
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }

        DrivetrainConfig.DATALOG_ENABLED = true
        RobotConfig.DATALOG_ENABLED = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_TUNING = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION = true
        DrivetrainConfig.ENABLE_CURRENT_TELEMETRY = true
        DrivetrainConfig.ENABLE_VISION_TELEMETRY = false
        DrivetrainConfig.ENABLE_EXTENDED_IMU_TELEMETRY = true

        phase = TestPhase.IDLE
        currentCycle = 0
        currentStepInCycle = 0
        globalStepIndex = 0
        cumulativeDistanceMeters = 0.0
        abortReason = null
        savedFileName = null
        savedRowCount = 0
        turnIntegral = 0.0
        lastTurnErrorRad = 0.0
        lastTurnOutput = 0.0

        telemetry.addLine("Drivetrain Figure-8 Loop Auto Datalogger")
        telemetry.addData("Test Plan", "%d Cycles x 16 Steps = %d Total Actions (%.1fm driving + 80 turns)",
            DrivetrainFigureEightConfig.TOTAL_CYCLES,
            DrivetrainFigureEightConfig.TOTAL_CYCLES * 16,
            DrivetrainFigureEightConfig.TOTAL_CYCLES * 8 * DrivetrainFigureEightConfig.SEGMENT_DISTANCE_METERS
        )
        telemetry.addLine("Ready: Press PLAY on Driver Station (or [A] on Gamepad) to begin")
        telemetry.addLine("Safety: Press STOP on Driver Station (or [B] on Gamepad) to abort")
        telemetry.update()
    }

    override fun start() {
        drivetrain.resetHeading(0.0)
        loopTimer.reset()
        if (DrivetrainFigureEightConfig.AUTO_START_ON_PLAY && phase == TestPhase.IDLE) {
            beginTest()
        }
    }

    override fun loop() {
        lynxModules.forEach { it.clearBulkCache() }
        val dtSec = loopTimer.seconds()
        loopTimer.reset()

        val nowMs = System.currentTimeMillis()
        val batteryVoltage = voltageSensor.voltage

        if (nowMs - lastTempQueryTimeMs >= 2000L || cachedHubTemps.isEmpty()) {
            lastTempQueryTimeMs = nowMs
            cachedHubTemps = lynxModules.map { hub ->
                try {
                    hub.getTemperature(TempUnit.CELSIUS)
                } catch (_: Exception) {
                    Double.NaN
                }
            }
        }

        val btnA = (gamepad1.a && !gamepad1.start) || (gamepad2.a && !gamepad2.start) || gamepad1.cross || gamepad2.cross
        val btnB = (gamepad1.b && !gamepad1.start) || (gamepad2.b && !gamepad2.start) || gamepad1.circle || gamepad2.circle || gamepad1.dpad_down || gamepad2.dpad_down

        if (btnA && !previousA && phase == TestPhase.IDLE) {
            beginTest()
        }

        if (btnB && !previousB && phase !in setOf(TestPhase.IDLE, TestPhase.COMPLETED, TestPhase.ABORTED)) {
            abortTest("Aborted by user button press")
        }

        previousA = btnA
        previousB = btnB

        var commandedForward = 0.0
        var commandedTurn = 0.0
        var holdHeadingInDrive = true

        val currentStep = if (currentStepInCycle in cycleSteps.indices) cycleSteps[currentStepInCycle] else null

        when (phase) {
            TestPhase.IDLE -> {
                if (DrivetrainFigureEightConfig.ENABLE_MANUAL_DRIVE_WHEN_IDLE) {
                    commandedForward = gamepad1.left_stick_y.toDouble()
                    commandedTurn = gamepad1.right_stick_x.toDouble()
                }
            }

            TestPhase.DRIVING -> {
                if (currentStep != null) {
                    val targetDist = DrivetrainFigureEightConfig.SEGMENT_DISTANCE_METERS
                    val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
                    val leftDeltaM = abs(segmentStartLeftTicks - drivetrain.getLeftEncoderPosition()) * (mmPerTick / 1000.0)
                    val rightDeltaM = abs(segmentStartRightTicks - drivetrain.getRightEncoderPosition()) * (mmPerTick / 1000.0)
                    segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0
                    segmentRemainingMeters = max(0.0, targetDist - segmentProgressMeters)

                    if (stepTimer.seconds() >= DrivetrainFigureEightConfig.STEP_TIMEOUT_SECONDS) {
                        abortTest("Drive step timed out (%.1fs > %.1fs)".format(stepTimer.seconds(), DrivetrainFigureEightConfig.STEP_TIMEOUT_SECONDS))
                    } else if (segmentProgressMeters >= targetDist) {
                        cumulativeDistanceMeters += segmentProgressMeters
                        phase = TestPhase.DRIVE_SETTLING
                        settleTimer.reset()
                        commandedForward = 0.0
                    } else {
                        commandedForward = calculateDrivePower(
                            progress = segmentProgressMeters,
                            remaining = segmentRemainingMeters,
                            total = targetDist
                        )
                    }
                }
            }

            TestPhase.DRIVE_SETTLING -> {
                commandedForward = 0.0
                commandedTurn = 0.0
                if (settleTimer.seconds() >= DrivetrainFigureEightConfig.DRIVE_SETTLE_SECONDS) {
                    advanceToNextStep()
                }
            }

            TestPhase.TURNING -> {
                if (currentStep != null) {
                    holdHeadingInDrive = false
                    val currentHeadingDeg = drivetrain.imu.robotYawPitchRollAngles.getYaw(AngleUnit.DEGREES)
                    val yawRateDegS = drivetrain.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate.toDouble()
                    val targetHeadingDeg = currentStep.targetHeadingDeg
                    val errorDeg = AngleUnit.normalizeDegrees(targetHeadingDeg - currentHeadingDeg)

                    val isAtTarget = abs(errorDeg) <= DrivetrainFigureEightConfig.TURN_TOLERANCE_DEG
                    val isStopped = abs(yawRateDegS) <= DrivetrainFigureEightConfig.TURN_MAX_VELOCITY_DEG_S

                    if (stepTimer.seconds() >= DrivetrainFigureEightConfig.STEP_TIMEOUT_SECONDS) {
                        abortTest("Turn step timed out (%.1fs > %.1fs) error=%.1f deg".format(stepTimer.seconds(), DrivetrainFigureEightConfig.STEP_TIMEOUT_SECONDS, errorDeg))
                    } else if (isAtTarget && isStopped) {
                        phase = TestPhase.TURN_SETTLING
                        settleTimer.reset()
                        commandedTurn = 0.0
                    } else {
                        // Calculate Turn PID power (negative turn commands CCW/Left, positive commands CW/Right)
                        val pidOutput = calculateTurnPID(errorDeg, dtSec)
                        lastTurnOutput = pidOutput
                        // In driveDirect: turn > 0 is CW (Right), turn < 0 is CCW (Left)
                        // When errorDeg > 0 (needs to turn CCW/Left), pass negative turn to driveDirect
                        commandedTurn = -pidOutput
                    }
                }
            }

            TestPhase.TURN_SETTLING -> {
                holdHeadingInDrive = false
                commandedForward = 0.0
                commandedTurn = 0.0
                if (settleTimer.seconds() >= DrivetrainFigureEightConfig.TURN_SETTLE_SECONDS) {
                    // Lock target heading for the next driving step
                    if (currentStep != null) {
                        drivetrain.setCustomHeadingDeg(currentStep.targetHeadingDeg)
                    }
                    advanceToNextStep()
                }
            }

            TestPhase.COMPLETED, TestPhase.ABORTED -> {
                commandedForward = 0.0
                commandedTurn = 0.0
            }
        }

        val driveTelemetry = drivetrain.driveDirect(
            forward = commandedForward,
            turn = commandedTurn,
            batteryVoltage = batteryVoltage,
            holdHeading = holdHeadingInDrive
        )

        if (datalogger.isLogging && driveTelemetry != null) {
            val stepLabel = currentStep?.description ?: phase.name
            val dirLabel = when (phase) {
                TestPhase.DRIVING -> "FORWARD"
                TestPhase.TURNING -> if ((currentStep?.targetHeadingDeg ?: 0.0) < 0) "TURN_RIGHT" else "TURN_LEFT"
                TestPhase.DRIVE_SETTLING, TestPhase.TURN_SETTLING -> "SETTLE"
                TestPhase.COMPLETED -> "COMPLETE"
                TestPhase.ABORTED -> "ABORT"
                else -> "IDLE"
            }

            datalogger.writeRow(
                testPhase = "C${currentCycle}_S${currentStepInCycle + 1}_$stepLabel",
                cycleIndex = currentCycle,
                totalCycles = DrivetrainFigureEightConfig.TOTAL_CYCLES,
                segmentIndex = globalStepIndex,
                segmentDirection = dirLabel,
                segmentTargetDistanceM = currentStep?.targetDistanceM ?: 0.0,
                segmentProgressM = segmentProgressMeters,
                segmentRemainingM = segmentRemainingMeters,
                totalDistanceM = cumulativeDistanceMeters + segmentProgressMeters,
                commandedForwardPwr = commandedForward,
                commandedTurnPwr = commandedTurn,
                drive = driveTelemetry,
                hubTemperaturesCelsius = cachedHubTemps,
                gamepad1 = gamepad1
            )
        }

        val packet = TelemetryPacket(false)
        packet.put("Phase", phase.name)
        packet.put("Cycle", "$currentCycle/${DrivetrainFigureEightConfig.TOTAL_CYCLES}")
        packet.put("Step", "${currentStepInCycle + 1}/16 (Global: $globalStepIndex/${DrivetrainFigureEightConfig.TOTAL_CYCLES * 16})")
        packet.put("Step Desc", currentStep?.description ?: "NONE")
        packet.put("Total Distance (m)", cumulativeDistanceMeters + segmentProgressMeters)
        packet.put("Battery (V)", batteryVoltage)

        if (DrivetrainConfig.ENABLE_ACTIVE_HEADING_TUNING) {
            val headingData: Drivetrain.HeadingCorrectionData = drivetrain.getHeadingCorrectionData()
            packet.put("Heading Target (deg)", headingData.targetHeadingDeg)
            packet.put("Heading Actual (deg)", headingData.currentHeadingDeg)
            packet.put("Heading Error (deg)", headingData.headingErrorDeg)
            packet.put("Correction Power", headingData.correctionPower)
            packet.put("Heading Active", headingData.isActive)
            packet.put("Heading Pending", headingData.isPending)
            packet.put("Tilt Safety", headingData.tiltingSafety)
            packet.put("Custom Heading (deg)", headingData.customHeadingDeg)
        }

        if (driveTelemetry != null) {
            packet.put("Heading (deg)", driveTelemetry.heading)
            packet.put("Target Heading (deg)", driveTelemetry.targetHeading)
            packet.put("Heading Error (deg)", driveTelemetry.headingError)
            packet.put("Yaw Rate (deg/s)", driveTelemetry.yawRate)
            packet.put("Heading PID Total", driveTelemetry.headingCorrection)
            packet.put("Left Pwr", driveTelemetry.leftMotorPower)
            packet.put("Right Pwr", driveTelemetry.rightMotorPower)
            packet.put("Left Current (A)", driveTelemetry.leftCurrentAmps)
            packet.put("Right Current (A)", driveTelemetry.rightCurrentAmps)

            val poseInches = Pose2d(
                Vector2d(driveTelemetry.poseX / 25.4, driveTelemetry.poseY / 25.4),
                Math.toRadians(driveTelemetry.poseHeading)
            )
            drawRobot(packet.fieldOverlay(), poseInches)
        }

        packet.put("Datalog Rows", datalogger.rowCount.takeIf { it > 0 } ?: savedRowCount)
        dashboard.sendTelemetryPacket(packet)

        renderTelemetry(driveTelemetry, currentStep)
    }

    private fun calculateDrivePower(progress: Double, remaining: Double, total: Double): Double {
        val minP = DrivetrainFigureEightConfig.MIN_DRIVE_POWER
        val maxP = DrivetrainFigureEightConfig.MAX_DRIVE_POWER
        val rampUp = DrivetrainFigureEightConfig.RAMP_UP_DISTANCE_METERS
        val rampDown = DrivetrainFigureEightConfig.RAMP_DOWN_DISTANCE_METERS

        val speed = when {
            progress < rampUp && rampUp > 0.0 -> {
                minP + (maxP - minP) * (progress / rampUp)
            }
            remaining < rampDown && rampDown > 0.0 -> {
                minP + (maxP - minP) * (remaining / rampDown)
            }
            else -> maxP
        }.coerceIn(minP, maxP)

        return DrivetrainFigureEightConfig.FORWARD_DIRECTION_SIGN * speed
    }

    private fun calculateTurnPID(errorDeg: Double, dtSec: Double): Double {
        val minP = DrivetrainFigureEightConfig.MIN_TURN_POWER
        val maxP = DrivetrainFigureEightConfig.MAX_TURN_POWER
        val kp = DrivetrainConfig.ACTIVE_HEADING_KP
        val kd = DrivetrainConfig.ACTIVE_HEADING_KD

        val errorRad = Math.toRadians(errorDeg)
        val yawRateRadPerSec = Math.toRadians(drivetrain.imu.getRobotAngularVelocity(AngleUnit.DEGREES).zRotationRate.toDouble())

        val p = errorRad * kp
        val d = -yawRateRadPerSec * kd

        var total = p + d
        if (abs(errorDeg) > 3.0 && abs(total) < minP) {
            val signVal = sign(total)
            total = signVal * minP
        }
        return total.coerceIn(-maxP, maxP)
    }

    private fun beginTest() {
        phase = TestPhase.DRIVING
        currentCycle = 1
        currentStepInCycle = 0
        globalStepIndex = 1
        cumulativeDistanceMeters = 0.0
        segmentProgressMeters = 0.0
        segmentRemainingMeters = DrivetrainFigureEightConfig.SEGMENT_DISTANCE_METERS
        abortReason = null

        drivetrain.resetHeading(0.0)
        recordSegmentStartTicks()
        stepTimer.reset()
        totalTestTimer.reset()
        turnIntegral = 0.0
        lastTurnErrorRad = 0.0

        datalogger.startLogging(prefix = "drivetrain_figure8_test")
        savedFileName = datalogger.fileName

        lynxModules.forEach { it.setPattern(runningPattern) }
    }

    private fun advanceToNextStep() {
        currentStepInCycle++
        globalStepIndex++

        if (currentStepInCycle >= cycleSteps.size) {
            currentStepInCycle = 0
            currentCycle++
        }

        if (currentCycle > DrivetrainFigureEightConfig.TOTAL_CYCLES) {
            completeTest()
            return
        }

        val nextStep = cycleSteps[currentStepInCycle]
        stepTimer.reset()
        turnIntegral = 0.0
        lastTurnErrorRad = 0.0

        if (nextStep.isTurn) {
            phase = TestPhase.TURNING
            drivetrain.setCustomHeadingDeg(nextStep.targetHeadingDeg)
            segmentProgressMeters = 0.0
            segmentRemainingMeters = 0.0
        } else {
            phase = TestPhase.DRIVING
            drivetrain.setCustomHeadingDeg(nextStep.targetHeadingDeg)
            recordSegmentStartTicks()
        }
    }

    private fun completeTest() {
        phase = TestPhase.COMPLETED
        drivetrain.stop()
        savedRowCount = datalogger.rowCount
        savedFileName = datalogger.fileName
        datalogger.stopLogging()

        lynxModules.forEach { it.setPattern(completePattern) }
    }

    private fun abortTest(reason: String) {
        phase = TestPhase.ABORTED
        abortReason = reason
        drivetrain.stop()
        savedRowCount = datalogger.rowCount
        savedFileName = datalogger.fileName
        datalogger.stopLogging()

        lynxModules.forEach { it.setPattern(abortPattern) }
    }

    private fun recordSegmentStartTicks() {
        segmentStartLeftTicks = drivetrain.getLeftEncoderPosition()
        segmentStartRightTicks = drivetrain.getRightEncoderPosition()
        segmentProgressMeters = 0.0
        segmentRemainingMeters = DrivetrainFigureEightConfig.SEGMENT_DISTANCE_METERS
    }

    private fun renderTelemetry(drive: fgc.vietnam.robot01.Hardware.DriveTelemetry?, step: StepDef?) {
        when (phase) {
            TestPhase.IDLE -> {
                telemetry.addLine("[READY] Press PLAY on Driver Station (or [A] on Gamepad)")
                telemetry.addData("Plan", "%d Cycles x 16 Steps = %d Actions (%.1f m driving + 80 turns)",
                    DrivetrainFigureEightConfig.TOTAL_CYCLES,
                    DrivetrainFigureEightConfig.TOTAL_CYCLES * 16,
                    DrivetrainFigureEightConfig.TOTAL_CYCLES * 8 * DrivetrainFigureEightConfig.SEGMENT_DISTANCE_METERS
                )
                telemetry.addData("Heading PID Gains", "kP=%.2f kI=%.2f kD=%.2f kS=%.2f deadband=%.1f deg",
                    DrivetrainConfig.ACTIVE_HEADING_KP,
                    DrivetrainConfig.ACTIVE_HEADING_KI,
                    DrivetrainConfig.ACTIVE_HEADING_KD,
                    DrivetrainConfig.ACTIVE_HEADING_KS,
                    DrivetrainConfig.ACTIVE_HEADING_HOLD_DEADBAND_DEG
                )
                telemetry.addData("Encoders", "L=%d, R=%d", drivetrain.getLeftEncoderPosition(), drivetrain.getRightEncoderPosition())
                if (savedFileName != null) {
                    telemetry.addLine("Previous Log: $savedFileName ($savedRowCount rows)")
                }
            }

            TestPhase.DRIVING, TestPhase.TURNING -> {
                val actionTag = if (phase == TestPhase.DRIVING) "[DRIVE]" else "[TURN]"
                telemetry.addLine("[RUNNING] $actionTag ${step?.description ?: ""}")
                telemetry.addData("Progress", "Cycle %d/%d | Step %d/16 (Global: %d/%d)",
                    currentCycle, DrivetrainFigureEightConfig.TOTAL_CYCLES,
                    currentStepInCycle + 1,
                    globalStepIndex, DrivetrainFigureEightConfig.TOTAL_CYCLES * 16
                )
                if (phase == TestPhase.DRIVING) {
                    telemetry.addData("Distance", "%.2f / %.2f m | Total: %.1f m",
                        segmentProgressMeters, DrivetrainFigureEightConfig.SEGMENT_DISTANCE_METERS,
                        cumulativeDistanceMeters + segmentProgressMeters
                    )
                }
                if (drive != null) {
                    telemetry.addData("Heading", "Now=%.1f deg Target=%.1f deg Error=%+.2f deg",
                        drive.heading, drive.targetHeading, drive.headingError
                    )
                    telemetry.addData("Heading PID", "P=%+.3f I=%+.3f D=%+.3f Total=%+.3f",
                        drive.proportionalCorrection, drive.integralCorrection, drive.derivativeCorrection, drive.headingCorrection
                    )
                    telemetry.addData("PID Gains", "kP=%.2f kI=%.2f kD=%.2f kS=%.2f",
                        DrivetrainConfig.ACTIVE_HEADING_KP,
                        DrivetrainConfig.ACTIVE_HEADING_KI,
                        DrivetrainConfig.ACTIVE_HEADING_KD,
                        DrivetrainConfig.ACTIVE_HEADING_KS
                    )
                    telemetry.addData("Motors", "L_Pwr=%.2f R_Pwr=%.2f | L_Vel=%.0f R_Vel=%.0f tps",
                        drive.leftMotorPower, drive.rightMotorPower,
                        drive.leftActualVelocity, drive.rightActualVelocity
                    )
                    telemetry.addData("Electrical", "V=%.2fV | L=%.2fA R=%.2fA (Tot=%.2fA)",
                        drive.batteryVoltage, drive.leftCurrentAmps, drive.rightCurrentAmps,
                        drive.leftCurrentAmps + drive.rightCurrentAmps
                    )
                }
                telemetry.addData("Datalog", "[REC] (%d rows) -> %s", datalogger.rowCount, datalogger.fileName ?: "logging...")
                telemetry.addLine("Press [B] to Abort")
            }

            TestPhase.DRIVE_SETTLING, TestPhase.TURN_SETTLING -> {
                telemetry.addLine("[SETTLING] Step %d/16 finished. Pausing...".format(currentStepInCycle + 1))
                telemetry.addData("Cycle", "%d / %d", currentCycle, DrivetrainFigureEightConfig.TOTAL_CYCLES)
            }

            TestPhase.COMPLETED -> {
                telemetry.addLine("[COMPLETE] All 10 Cycles (160 Steps) Finished Successfully!")
                telemetry.addData("Total Distance", "%.2f meters", cumulativeDistanceMeters)
                telemetry.addData("Total Test Time", "%.2f seconds", totalTestTimer.seconds())
                telemetry.addData("Datalog Saved", "%s (%d rows)", savedFileName ?: "saved", savedRowCount)
                telemetry.addLine("File saved in /sdcard/FIRST/Datalogs/ (Retrieve via Web UI / ADB)")
                telemetry.addLine("Press [A] to run again")
            }

            TestPhase.ABORTED -> {
                telemetry.addLine("[ABORTED] $abortReason")
                telemetry.addData("Completed Steps", "%d / %d", globalStepIndex, DrivetrainFigureEightConfig.TOTAL_CYCLES * 16)
                telemetry.addData("Partial Log Saved", "%s (%d rows)", savedFileName ?: "saved", savedRowCount)
                telemetry.addLine("Press [A] to restart test")
            }
        }
        telemetry.update()
    }

    override fun stop() {
        drivetrain.stop()
        if (datalogger.isLogging) {
            savedRowCount = datalogger.rowCount
            savedFileName = datalogger.fileName
            datalogger.stopLogging()
        }
    }
}
