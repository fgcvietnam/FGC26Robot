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
import FGC.Vietnam.Config.IntakeConfig
import FGC.Vietnam.Config.RobotConfig
import FGC.Vietnam.DataLogger.IntakeDatalogger
import fgc.vietnam.robot01.Hardware.Drivetrain
import fgc.vietnam.robot01.Hardware.Intake
import fgc.vietnam.robot01.Hardware.IntakeSlidePosition
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max

@Config
object IntakeAutoDatalog065Config {
    @JvmField var AUTO_START_ON_PLAY = true
    @JvmField var TARGET_DISTANCE_METERS = 3.0
    @JvmField var TOTAL_CYCLES = 1 // 1 Forward + 1 Backward = 2 segments total
    @JvmField var MAX_DRIVE_POWER = 0.65
    @JvmField var MIN_DRIVE_POWER = 0.28
    @JvmField var RAMP_UP_DISTANCE_METERS = 0.35
    @JvmField var RAMP_DOWN_DISTANCE_METERS = 0.75
    @JvmField var SETTLE_TIME_SECONDS = 0.40
    @JvmField var EXTENSION_TIMEOUT_SECONDS = 5.0
    @JvmField var SEGMENT_TIMEOUT_SECONDS = 15.0
    @JvmField var FORWARD_DIRECTION_SIGN = -1.0
    @JvmField var ENABLE_MANUAL_DRIVE_WHEN_IDLE = true
}

/**
 * Automated Intake Datalogger at Speed 0.65 (Autonomous).
 *
 * Sequence:
 * 1. Automatically extends the intake slides using CRServos until both magnetic sensors confirm full extension.
 * 2. Starts the intake roller motor.
 * 3. Executes 3.0m Forward and 3.0m Backward runs for the configured number of cycles.
 * 4. Logs all Intake, electrical, slide extension, sensors, and drivetrain kinematics to CSV.
 */
@Autonomous(name = "Intake Auto Datalog 3m (Speed 0.65)", group = "Autonomous")
class IntakeAutoDatalog065OpMode : OpMode() {

    enum class TestPhase {
        IDLE,
        EXTENDING_SLIDES,
        STARTING_INTAKE,
        FORWARD_RUN,
        FORWARD_SETTLE,
        BACKWARD_RUN,
        BACKWARD_SETTLE,
        COMPLETED,
        ABORTED
    }

    private lateinit var drivetrain: Drivetrain
    private lateinit var intake: Intake
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var dashboard: FtcDashboard

    private val datalogger = IntakeDatalogger()

    private var phase = TestPhase.IDLE
    private var currentCycle = 1
    private var segmentIndex = 1
    private var cumulativeDistanceMeters = 0.0

    private var segmentStartLeftTicks = 0
    private var segmentStartRightTicks = 0
    private var segmentProgressMeters = 0.0
    private var segmentRemainingMeters = 0.0

    private val segmentTimer = ElapsedTime()
    private val extensionTimer = ElapsedTime()
    private val totalTestTimer = ElapsedTime()
    private val settleTimer = ElapsedTime()

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
        intake = Intake(hardwareMap)
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }

        // Enable full datalogging and electrical telemetry
        DrivetrainConfig.DATALOG_ENABLED = true
        RobotConfig.DATALOG_ENABLED = true
        IntakeConfig.DATALOG_ENABLED = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_TUNING = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION = true
        DrivetrainConfig.ENABLE_CURRENT_TELEMETRY = true
        DrivetrainConfig.ENABLE_VISION_TELEMETRY = false
        DrivetrainConfig.ENABLE_EXTENDED_IMU_TELEMETRY = true

        phase = TestPhase.IDLE
        currentCycle = 1
        segmentIndex = 1
        cumulativeDistanceMeters = 0.0
        abortReason = null
        savedFileName = null
        savedRowCount = 0

        telemetry.addLine("Intake 3m Auto Datalogger (Speed 0.65)")
        telemetry.addData("Test Plan", "Extend Slides -> Intake Spin -> %.1fm Fwd/Bwd x %d cycles (%d segments)",
            IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS,
            IntakeAutoDatalog065Config.TOTAL_CYCLES,
            IntakeAutoDatalog065Config.TOTAL_CYCLES * 2
        )
        telemetry.addData("Speed & Power", "Max=%.2f, Min=%.2f",
            IntakeAutoDatalog065Config.MAX_DRIVE_POWER, IntakeAutoDatalog065Config.MIN_DRIVE_POWER
        )
        telemetry.addLine("Ready: Press PLAY on Driver Station (or [A] on Gamepad) to begin")
        telemetry.addLine("Safety: Press STOP on Driver Station (or [B] on Gamepad) to abort")
        telemetry.update()
    }

    override fun start() {
        drivetrain.resetHeading(0.0)
        if (IntakeAutoDatalog065Config.AUTO_START_ON_PLAY && phase == TestPhase.IDLE) {
            beginTest()
        }
    }

    override fun loop() {
        lynxModules.forEach { it.clearBulkCache() }
        intake.update()

        val nowMs = System.currentTimeMillis()
        val batteryVoltage = voltageSensor.voltage

        // Poll hub temperatures every 2 seconds
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

        // User Buttons (guarded against Start+A / Start+B)
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

        // State Machine Execution
        var commandedForward = 0.0
        var commandedTurn = 0.0

        when (phase) {
            TestPhase.IDLE -> {
                if (IntakeAutoDatalog065Config.ENABLE_MANUAL_DRIVE_WHEN_IDLE) {
                    commandedForward = gamepad1.left_stick_y.toDouble()
                    commandedTurn = gamepad1.right_stick_x.toDouble()
                }
            }

            TestPhase.EXTENDING_SLIDES -> {
                // Drive extension servos forward until magnetic switches detect end of slide travel
                intake.moveServosForward()
                commandedForward = 0.0
                commandedTurn = 0.0

                val isFullyExt = intake.isFullyExtended()
                val isTimedOut = extensionTimer.seconds() >= IntakeAutoDatalog065Config.EXTENSION_TIMEOUT_SECONDS

                if (isFullyExt || isTimedOut) {
                    intake.stopServos()
                    // Move to starting the intake motor
                    phase = TestPhase.STARTING_INTAKE
                    settleTimer.reset()
                }
            }

            TestPhase.STARTING_INTAKE -> {
                intake.intake() // Start intake motor spinning
                commandedForward = 0.0
                commandedTurn = 0.0

                if (settleTimer.seconds() >= 0.25) {
                    // Ready to begin forward driving segment
                    phase = TestPhase.FORWARD_RUN
                    segmentIndex = 1
                    currentCycle = 1
                    recordSegmentStartTicks()
                    segmentTimer.reset()
                }
            }

            TestPhase.FORWARD_RUN -> {
                val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
                val leftDeltaM = abs(segmentStartLeftTicks - getCurrentLeftTicks()) * (mmPerTick / 1000.0)
                val rightDeltaM = abs(segmentStartRightTicks - getCurrentRightTicks()) * (mmPerTick / 1000.0)
                segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0
                segmentRemainingMeters = max(0.0, IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS - segmentProgressMeters)

                val segmentTimeoutSec = max(IntakeAutoDatalog065Config.SEGMENT_TIMEOUT_SECONDS, (IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS / (IntakeAutoDatalog065Config.MAX_DRIVE_POWER * 0.6)) + 5.0)
                if (segmentTimer.seconds() >= segmentTimeoutSec) {
                    abortTest("Forward segment timed out (%.1fs > %.1fs)".format(segmentTimer.seconds(), segmentTimeoutSec))
                } else if (segmentProgressMeters >= IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.FORWARD_SETTLE
                    settleTimer.reset()
                    commandedForward = 0.0
                } else {
                    commandedForward = calculateProfilePower(
                        progress = segmentProgressMeters,
                        remaining = segmentRemainingMeters,
                        directionSign = 1.0
                    )
                }
            }

            TestPhase.FORWARD_SETTLE -> {
                commandedForward = 0.0
                if (settleTimer.seconds() >= IntakeAutoDatalog065Config.SETTLE_TIME_SECONDS) {
                    phase = TestPhase.BACKWARD_RUN
                    segmentIndex++
                    recordSegmentStartTicks()
                    segmentTimer.reset()
                }
            }

            TestPhase.BACKWARD_RUN -> {
                val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
                val leftDeltaM = abs(segmentStartLeftTicks - getCurrentLeftTicks()) * (mmPerTick / 1000.0)
                val rightDeltaM = abs(segmentStartRightTicks - getCurrentRightTicks()) * (mmPerTick / 1000.0)
                segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0
                segmentRemainingMeters = max(0.0, IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS - segmentProgressMeters)

                val segmentTimeoutSec = max(IntakeAutoDatalog065Config.SEGMENT_TIMEOUT_SECONDS, (IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS / (IntakeAutoDatalog065Config.MAX_DRIVE_POWER * 0.6)) + 5.0)
                if (segmentTimer.seconds() >= segmentTimeoutSec) {
                    abortTest("Backward segment timed out (%.1fs > %.1fs)".format(segmentTimer.seconds(), segmentTimeoutSec))
                } else if (segmentProgressMeters >= IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.BACKWARD_SETTLE
                    settleTimer.reset()
                    commandedForward = 0.0
                } else {
                    commandedForward = calculateProfilePower(
                        progress = segmentProgressMeters,
                        remaining = segmentRemainingMeters,
                        directionSign = -1.0
                    )
                }
            }

            TestPhase.BACKWARD_SETTLE -> {
                commandedForward = 0.0
                if (settleTimer.seconds() >= IntakeAutoDatalog065Config.SETTLE_TIME_SECONDS) {
                    if (currentCycle < IntakeAutoDatalog065Config.TOTAL_CYCLES) {
                        currentCycle++
                        segmentIndex++
                        phase = TestPhase.FORWARD_RUN
                        recordSegmentStartTicks()
                        segmentTimer.reset()
                    } else {
                        completeTest()
                    }
                }
            }

            TestPhase.COMPLETED, TestPhase.ABORTED -> {
                commandedForward = 0.0
                commandedTurn = 0.0
                intake.stopMotor()
                intake.stopServos()
            }
        }

        // Apply Drivetrain Direct Drive
        val driveTelemetry = drivetrain.driveDirect(
            forward = commandedForward,
            turn = commandedTurn,
            batteryVoltage = batteryVoltage,
            holdHeading = DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION
        )

        // Datalogging writeRow
        if (datalogger.isLogging && driveTelemetry != null) {
            val directionLabel = when (phase) {
                TestPhase.EXTENDING_SLIDES -> "EXTEND_SLIDES"
                TestPhase.STARTING_INTAKE -> "START_INTAKE"
                TestPhase.FORWARD_RUN -> "FORWARD"
                TestPhase.BACKWARD_RUN -> "BACKWARD"
                TestPhase.FORWARD_SETTLE, TestPhase.BACKWARD_SETTLE -> "SETTLE"
                TestPhase.COMPLETED -> "COMPLETE"
                TestPhase.ABORTED -> "ABORT"
                else -> "IDLE"
            }

            datalogger.writeRow(
                testPhase = "C${currentCycle}_${phase.name}",
                cycleIndex = currentCycle,
                totalCycles = IntakeAutoDatalog065Config.TOTAL_CYCLES,
                segmentIndex = segmentIndex,
                segmentDirection = directionLabel,
                segmentTargetDistanceM = IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS,
                segmentProgressM = segmentProgressMeters,
                segmentRemainingM = segmentRemainingMeters,
                totalDistanceM = cumulativeDistanceMeters + segmentProgressMeters,
                commandedForwardPwr = commandedForward,
                commandedTurnPwr = commandedTurn,
                drive = driveTelemetry,
                intake = intake,
                hubTemperaturesCelsius = cachedHubTemps,
                gamepad1 = gamepad1
            )
        }

        // FTC Dashboard Packet
        val packet = TelemetryPacket(false)
        packet.put("Phase", phase.name)
        packet.put("Cycle", "$currentCycle/${IntakeAutoDatalog065Config.TOTAL_CYCLES}")
        packet.put("Segment", "$segmentIndex/${IntakeAutoDatalog065Config.TOTAL_CYCLES * 2}")
        packet.put("Segment Progress (m)", segmentProgressMeters)
        packet.put("Total Distance (m)", cumulativeDistanceMeters + segmentProgressMeters)
        packet.put("Commanded Forward Pwr", commandedForward)
        packet.put("Battery (V)", batteryVoltage)

        val inTel = intake.getTelemetry()
        packet.put("Intake State", inTel.state.name)
        packet.put("Left Slide", inTel.leftSlidePosition.name)
        packet.put("Right Slide", inTel.rightSlidePosition.name)
        packet.put("Left Magnet", inTel.leftMagneticSwitchPressed)
        packet.put("Right Magnet", inTel.rightMagneticSwitchPressed)
        packet.put("Both Magnets", inTel.bothMagnetsDetected)
        packet.put("Intake Current (A)", inTel.motorCurrentAmps)
        packet.put("Transfer Current (A)", inTel.hexMotorCurrentAmps)

        if (driveTelemetry != null) {
            packet.put("Heading (deg)", driveTelemetry.heading)
            packet.put("Heading Error (deg)", driveTelemetry.headingError)
            packet.put("Linear Speed (m/s)", driveTelemetry.linearSpeedMmPerSecond / 1000.0)
            packet.put("Drive Current (A)", driveTelemetry.leftCurrentAmps + driveTelemetry.rightCurrentAmps)

            val poseInches = Pose2d(
                Vector2d(driveTelemetry.poseX / 25.4, driveTelemetry.poseY / 25.4),
                Math.toRadians(driveTelemetry.poseHeading)
            )
            drawRobot(packet.fieldOverlay(), poseInches)
        }

        packet.put("Datalog Rows", datalogger.rowCount.takeIf { it > 0 } ?: savedRowCount)
        dashboard.sendTelemetryPacket(packet)

        // Driver Station Telemetry
        renderTelemetry(driveTelemetry)
    }

    private fun calculateProfilePower(
        progress: Double,
        remaining: Double,
        directionSign: Double
    ): Double {
        val minP = IntakeAutoDatalog065Config.MIN_DRIVE_POWER
        val maxP = IntakeAutoDatalog065Config.MAX_DRIVE_POWER
        val rampUp = IntakeAutoDatalog065Config.RAMP_UP_DISTANCE_METERS
        val rampDown = IntakeAutoDatalog065Config.RAMP_DOWN_DISTANCE_METERS

        val speed = when {
            progress < rampUp && rampUp > 0.0 -> {
                minP + (maxP - minP) * (progress / rampUp)
            }
            remaining < rampDown && rampDown > 0.0 -> {
                minP + (maxP - minP) * (remaining / rampDown)
            }
            else -> maxP
        }.coerceIn(minP, maxP)

        return directionSign * IntakeAutoDatalog065Config.FORWARD_DIRECTION_SIGN * speed
    }

    private fun beginTest() {
        phase = TestPhase.EXTENDING_SLIDES
        currentCycle = 1
        segmentIndex = 1
        cumulativeDistanceMeters = 0.0
        segmentProgressMeters = 0.0
        segmentRemainingMeters = IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS
        abortReason = null

        drivetrain.resetHeading(0.0)
        extensionTimer.reset()
        totalTestTimer.reset()

        datalogger.startLogging(prefix = "intake_3m_spd065_test")
        savedFileName = datalogger.fileName

        lynxModules.forEach { it.setPattern(runningPattern) }
    }

    private fun completeTest() {
        phase = TestPhase.COMPLETED
        drivetrain.stop()
        intake.stopMotor()
        intake.stopServos()
        savedRowCount = datalogger.rowCount
        savedFileName = datalogger.fileName
        datalogger.stopLogging()

        lynxModules.forEach { it.setPattern(completePattern) }
    }

    private fun abortTest(reason: String) {
        phase = TestPhase.ABORTED
        abortReason = reason
        drivetrain.stop()
        intake.stopMotor()
        intake.stopServos()
        savedRowCount = datalogger.rowCount
        savedFileName = datalogger.fileName
        datalogger.stopLogging()

        lynxModules.forEach { it.setPattern(abortPattern) }
    }

    private fun recordSegmentStartTicks() {
        segmentStartLeftTicks = drivetrain.getLeftEncoderPosition()
        segmentStartRightTicks = drivetrain.getRightEncoderPosition()
        segmentProgressMeters = 0.0
        segmentRemainingMeters = IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS
    }

    private fun getCurrentLeftTicks(): Int = drivetrain.getLeftEncoderPosition()

    private fun getCurrentRightTicks(): Int = drivetrain.getRightEncoderPosition()

    private fun renderTelemetry(drive: fgc.vietnam.robot01.Hardware.DriveTelemetry?) {
        val totalSegments = IntakeAutoDatalog065Config.TOTAL_CYCLES * 2
        when (phase) {
            TestPhase.IDLE -> {
                telemetry.addLine("[READY] Intake 3m Datalogger (Speed 0.65)")
                telemetry.addData("Plan", "%d Cycles = %d Segments (%.1fm total)",
                    IntakeAutoDatalog065Config.TOTAL_CYCLES, totalSegments,
                    IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS * totalSegments
                )
                telemetry.addData("Intake Extension", "L_Mag=%b, R_Mag=%b, Both=%b",
                    intake.leftMagnetRegistered(), intake.rightMagnetRegistered(), intake.isFullyExtended()
                )
                if (savedFileName != null) {
                    telemetry.addLine("Previous Log: $savedFileName ($savedRowCount rows)")
                }
            }

            TestPhase.EXTENDING_SLIDES -> {
                telemetry.addLine("[EXTENDING] Moving Servos Forward to Magnets...")
                telemetry.addData("Time", "%.2fs / %.1fs", extensionTimer.seconds(), IntakeAutoDatalog065Config.EXTENSION_TIMEOUT_SECONDS)
                telemetry.addData("Magnets", "L=%b, R=%b | State=%s",
                    intake.leftMagnetRegistered(), intake.rightMagnetRegistered(), intake.getMagnetSyncState()
                )
            }

            TestPhase.STARTING_INTAKE -> {
                telemetry.addLine("[STARTING INTAKE] Spinning intake roller...")
            }

            TestPhase.FORWARD_RUN, TestPhase.BACKWARD_RUN -> {
                val dir = if (phase == TestPhase.FORWARD_RUN) "FORWARD >>" else "<< BACKWARD"
                telemetry.addLine("[RUNNING] $dir | INTAKE ON (Spd 0.65)")
                telemetry.addData("Cycle", "%d / %d  |  Segment: %d / %d",
                    currentCycle, IntakeAutoDatalog065Config.TOTAL_CYCLES,
                    segmentIndex, totalSegments
                )
                telemetry.addData("Progress", "%.2f / %.2f m (%.1f%%)",
                    segmentProgressMeters, IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS,
                    (segmentProgressMeters / IntakeAutoDatalog065Config.TARGET_DISTANCE_METERS) * 100.0
                )
                if (drive != null) {
                    telemetry.addData("Speed & Current", "%.2f m/s | Drive=%.1fA, Intake=%.1fA (Total=%.1fA)",
                        drive.linearSpeedMmPerSecond / 1000.0,
                        drive.leftCurrentAmps + drive.rightCurrentAmps,
                        intake.getTelemetry().motorCurrentAmps,
                        drive.leftCurrentAmps + drive.rightCurrentAmps + intake.getTelemetry().motorCurrentAmps
                    )
                    telemetry.addData("Heading", "Now=%.1f deg Target=%.1f deg Error=%+.2f deg",
                        drive.heading, drive.targetHeading, drive.headingError
                    )
                }
                telemetry.addData("Datalog", "[REC] (%d rows) -> %s", datalogger.rowCount, datalogger.fileName ?: "logging...")
                telemetry.addLine("Press [B] to Abort")
            }

            TestPhase.FORWARD_SETTLE, TestPhase.BACKWARD_SETTLE -> {
                telemetry.addLine("[SETTLING] Pausing %.2fs between passes...".format(IntakeAutoDatalog065Config.SETTLE_TIME_SECONDS))
                telemetry.addData("Cycle", "%d / %d finished", currentCycle, IntakeAutoDatalog065Config.TOTAL_CYCLES)
            }

            TestPhase.COMPLETED -> {
                telemetry.addLine("[COMPLETE] Intake 3m Test Finished Successfully!")
                telemetry.addData("Total Distance", "%.2f meters", cumulativeDistanceMeters)
                telemetry.addData("Total Time", "%.2f seconds", totalTestTimer.seconds())
                telemetry.addData("Datalog Saved", "%s (%d rows)", savedFileName ?: "saved", savedRowCount)
                telemetry.addLine("File saved in /sdcard/FIRST/Datalogs/")
                telemetry.addLine("Press [A] to run again")
            }

            TestPhase.ABORTED -> {
                telemetry.addLine("[ABORTED] $abortReason")
                telemetry.addData("Distance before abort", "%.2f meters", cumulativeDistanceMeters + segmentProgressMeters)
                telemetry.addData("Partial Log Saved", "%s (%d rows)", savedFileName ?: "saved", savedRowCount)
                telemetry.addLine("Press [A] to restart test")
            }
        }
        telemetry.update()
    }

    override fun stop() {
        drivetrain.stop()
        intake.stopMotor()
        intake.stopServos()
        if (datalogger.isLogging) {
            savedRowCount = datalogger.rowCount
            savedFileName = datalogger.fileName
            datalogger.stopLogging()
        }
    }
}
