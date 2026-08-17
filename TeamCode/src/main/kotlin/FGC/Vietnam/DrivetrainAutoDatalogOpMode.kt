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
import FGC.Vietnam.Config.RoadRunnerConfig
import FGC.Vietnam.Config.RobotConfig
import FGC.Vietnam.DataLogger.DrivetrainDatalogger
import fgc.vietnam.robot01.Hardware.Drivetrain
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.Locale
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@Config
object DrivetrainAutoDatalogConfig {
    @JvmField var AUTO_START_ON_PLAY = true
    @JvmField var TARGET_DISTANCE_METERS = 3.0
    @JvmField var TOTAL_CYCLES = 10 // 10 Forward + 10 Backward = 20 segments
    @JvmField var MAX_DRIVE_POWER = 0.70
    @JvmField var MIN_DRIVE_POWER = 0.35 // Minimum power to reliably overcome static friction
    @JvmField var RAMP_UP_DISTANCE_METERS = 0.35
    @JvmField var RAMP_DOWN_DISTANCE_METERS = 0.55
    @JvmField var SETTLE_TIME_SECONDS = 0.5
    @JvmField var SEGMENT_TIMEOUT_SECONDS = 15.0
    @JvmField var FORWARD_DIRECTION_SIGN = 1.0 // Change to -1.0 if robot forward direction is inverted
    @JvmField var ENABLE_MANUAL_DRIVE_WHEN_IDLE = true
}

/**
 * Automated Drivetrain Datalogging Autonomous OpMode.
 *
 * Runs a high-precision back-and-forth routine:
 * - 3.0 meters Forward, 3.0 meters Backward
 * - Repeated 10 times (10 Forward + 10 Backward = 20 runs total)
 * - Automatically starts datalogging on test trigger / DS Play
 * - Uses direct power (driveDirect) to eliminate joystick deadband clipping
 * - Automatically flushes & saves the CSV file on completion or abort
 * - Captures 76 comprehensive drivetrain, dynamics, electrical, kinematics, and IMU metrics
 */
@Autonomous(name = "Drivetrain Auto Datalog (3m x 10 F/B)", group = "Autonomous")
class DrivetrainAutoDatalogOpMode : OpMode() {

    enum class TestPhase {
        IDLE,
        FORWARD_RUN,
        FORWARD_SETTLE,
        BACKWARD_RUN,
        BACKWARD_SETTLE,
        COMPLETED,
        ABORTED
    }

    private lateinit var drivetrain: Drivetrain
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var dashboard: FtcDashboard

    private val datalogger = DrivetrainDatalogger()

    private var phase = TestPhase.IDLE
    private var currentCycle = 0 // 1..TOTAL_CYCLES
    private var segmentIndex = 0 // 1..(TOTAL_CYCLES * 2)
    private var cumulativeDistanceMeters = 0.0

    private var segmentStartLeftTicks = 0
    private var segmentStartRightTicks = 0
    private var segmentProgressMeters = 0.0
    private var segmentRemainingMeters = 0.0

    private val segmentTimer = ElapsedTime()
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
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }

        // Ensure datalogging telemetry flag and active heading tuning are active in drivetrain
        DrivetrainConfig.DATALOG_ENABLED = true
        RobotConfig.DATALOG_ENABLED = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_TUNING = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION = true

        phase = TestPhase.IDLE
        currentCycle = 0
        segmentIndex = 0
        cumulativeDistanceMeters = 0.0
        abortReason = null
        savedFileName = null
        savedRowCount = 0

        telemetry.addLine("Drivetrain 3m x 10 Auto Datalogger (Autonomous)")
        telemetry.addData("Test Plan", "%.1fm Fwd + %.1fm Bwd x %d cycles (%d segments = %.1fm total)",
            DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS,
            DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS,
            DrivetrainAutoDatalogConfig.TOTAL_CYCLES,
            DrivetrainAutoDatalogConfig.TOTAL_CYCLES * 2,
            DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS * DrivetrainAutoDatalogConfig.TOTAL_CYCLES * 2.0
        )
        telemetry.addLine("Ready: Press PLAY on Driver Station (or [A] on Gamepad) to begin")
        telemetry.addLine("Safety: Press STOP on Driver Station (or [B] on Gamepad) to abort")
        telemetry.update()
    }

    override fun start() {
        drivetrain.resetHeading(0.0)
        if (DrivetrainAutoDatalogConfig.AUTO_START_ON_PLAY && phase == TestPhase.IDLE) {
            beginTest()
        }
    }

    override fun loop() {
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

        // User Buttons (guarded against Start+A / Start+B controller assignment)
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
                if (DrivetrainAutoDatalogConfig.ENABLE_MANUAL_DRIVE_WHEN_IDLE) {
                    commandedForward = -gamepad1.left_stick_y.toDouble()
                    commandedTurn = gamepad1.right_stick_x.toDouble()
                }
            }

            TestPhase.FORWARD_RUN -> {
                val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
                val leftDeltaM = abs(segmentStartLeftTicks - getCurrentLeftTicks()) * (mmPerTick / 1000.0)
                val rightDeltaM = abs(segmentStartRightTicks - getCurrentRightTicks()) * (mmPerTick / 1000.0)
                segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0
                segmentRemainingMeters = max(0.0, DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS - segmentProgressMeters)

                // Segment Watchdog
                if (segmentTimer.seconds() >= DrivetrainAutoDatalogConfig.SEGMENT_TIMEOUT_SECONDS) {
                    abortTest("Forward segment timed out (%.1fs > %.1fs) - possible stall/wheel slip"
                        .format(segmentTimer.seconds(), DrivetrainAutoDatalogConfig.SEGMENT_TIMEOUT_SECONDS))
                } else if (segmentProgressMeters >= DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS) {
                    // Reached 3m forward -> enter forward settle phase
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.FORWARD_SETTLE
                    settleTimer.reset()
                    commandedForward = 0.0
                } else {
                    commandedForward = calculateProfilePower(
                        progress = segmentProgressMeters,
                        remaining = segmentRemainingMeters,
                        total = DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS,
                        directionSign = 1.0
                    )
                }
            }

            TestPhase.FORWARD_SETTLE -> {
                commandedForward = 0.0
                if (settleTimer.seconds() >= DrivetrainAutoDatalogConfig.SETTLE_TIME_SECONDS) {
                    // Settle complete -> Start Backward pass
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
                segmentRemainingMeters = max(0.0, DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS - segmentProgressMeters)

                // Segment Watchdog
                if (segmentTimer.seconds() >= DrivetrainAutoDatalogConfig.SEGMENT_TIMEOUT_SECONDS) {
                    abortTest("Backward segment timed out (%.1fs > %.1fs) - possible stall/wheel slip"
                        .format(segmentTimer.seconds(), DrivetrainAutoDatalogConfig.SEGMENT_TIMEOUT_SECONDS))
                } else if (segmentProgressMeters >= DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS) {
                    // Reached 3m backward -> enter backward settle phase
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.BACKWARD_SETTLE
                    settleTimer.reset()
                    commandedForward = 0.0
                } else {
                    commandedForward = calculateProfilePower(
                        progress = segmentProgressMeters,
                        remaining = segmentRemainingMeters,
                        total = DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS,
                        directionSign = -1.0
                    )
                }
            }

            TestPhase.BACKWARD_SETTLE -> {
                commandedForward = 0.0
                if (settleTimer.seconds() >= DrivetrainAutoDatalogConfig.SETTLE_TIME_SECONDS) {
                    if (currentCycle < DrivetrainAutoDatalogConfig.TOTAL_CYCLES) {
                        // Advance to next cycle's forward run
                        currentCycle++
                        segmentIndex++
                        phase = TestPhase.FORWARD_RUN
                        recordSegmentStartTicks()
                        segmentTimer.reset()
                    } else {
                        // All cycles finished successfully!
                        completeTest()
                    }
                }
            }

            TestPhase.COMPLETED, TestPhase.ABORTED -> {
                commandedForward = 0.0
                commandedTurn = 0.0
            }
        }

        // Apply Drivetrain Direct Drive (bypasses joystick deadband and expo)
        val driveTelemetry = drivetrain.driveDirect(
            forward = commandedForward,
            turn = commandedTurn,
            batteryVoltage = batteryVoltage,
            holdHeading = DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION
        )

        // Datalogging (automatic write every loop during test or manual log)
        if (datalogger.isLogging && driveTelemetry != null) {
            val directionLabel = when (phase) {
                TestPhase.FORWARD_RUN -> "FORWARD"
                TestPhase.BACKWARD_RUN -> "BACKWARD"
                TestPhase.FORWARD_SETTLE, TestPhase.BACKWARD_SETTLE -> "SETTLE"
                TestPhase.COMPLETED -> "COMPLETE"
                TestPhase.ABORTED -> "ABORT"
                else -> "IDLE"
            }

            datalogger.writeRow(
                testPhase = phase.name,
                cycleIndex = currentCycle,
                totalCycles = DrivetrainAutoDatalogConfig.TOTAL_CYCLES,
                segmentIndex = segmentIndex,
                segmentDirection = directionLabel,
                segmentTargetDistanceM = DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS,
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

        // FTC Dashboard Packet
        val packet = TelemetryPacket(false)
        packet.put("Phase", phase.name)
        packet.put("Cycle", "$currentCycle/${DrivetrainAutoDatalogConfig.TOTAL_CYCLES}")
        packet.put("Segment", "$segmentIndex/${DrivetrainAutoDatalogConfig.TOTAL_CYCLES * 2}")
        packet.put("Segment Progress (m)", segmentProgressMeters)
        packet.put("Total Distance (m)", cumulativeDistanceMeters + segmentProgressMeters)
        packet.put("Commanded Forward Pwr", commandedForward)
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
            packet.put("Left Velocity (ticks/s)", driveTelemetry.leftActualVelocity)
            packet.put("Right Velocity (ticks/s)", driveTelemetry.rightActualVelocity)
            packet.put("Left Current (A)", driveTelemetry.leftCurrentAmps)
            packet.put("Right Current (A)", driveTelemetry.rightCurrentAmps)
            packet.put("Total Current (A)", driveTelemetry.leftCurrentAmps + driveTelemetry.rightCurrentAmps)

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
        total: Double,
        directionSign: Double
    ): Double {
        val minP = DrivetrainAutoDatalogConfig.MIN_DRIVE_POWER
        val maxP = DrivetrainAutoDatalogConfig.MAX_DRIVE_POWER
        val rampUp = DrivetrainAutoDatalogConfig.RAMP_UP_DISTANCE_METERS
        val rampDown = DrivetrainAutoDatalogConfig.RAMP_DOWN_DISTANCE_METERS

        val speed = when {
            progress < rampUp && rampUp > 0.0 -> {
                minP + (maxP - minP) * (progress / rampUp)
            }
            remaining < rampDown && rampDown > 0.0 -> {
                minP + (maxP - minP) * (remaining / rampDown)
            }
            else -> maxP
        }.coerceIn(minP, maxP)

        return directionSign * DrivetrainAutoDatalogConfig.FORWARD_DIRECTION_SIGN * speed
    }

    private fun beginTest() {
        phase = TestPhase.FORWARD_RUN
        currentCycle = 1
        segmentIndex = 1
        cumulativeDistanceMeters = 0.0
        segmentProgressMeters = 0.0
        segmentRemainingMeters = DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS
        abortReason = null

        drivetrain.resetHeading(0.0)
        recordSegmentStartTicks()
        segmentTimer.reset()
        totalTestTimer.reset()

        // Start datalogging session
        datalogger.startLogging(prefix = "drivetrain_3m_test")
        savedFileName = datalogger.fileName

        lynxModules.forEach { it.setPattern(runningPattern) }
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
        segmentRemainingMeters = DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS
    }

    private fun getCurrentLeftTicks(): Int = drivetrain.getLeftEncoderPosition()

    private fun getCurrentRightTicks(): Int = drivetrain.getRightEncoderPosition()

    private fun renderTelemetry(drive: fgc.vietnam.robot01.Hardware.DriveTelemetry?) {
        when (phase) {
            TestPhase.IDLE -> {
                telemetry.addLine("[READY] Press PLAY on Driver Station (or [A] on Gamepad)")
                telemetry.addData("Plan", "%d Cycles = %d Segments (%.1f m total)",
                    DrivetrainAutoDatalogConfig.TOTAL_CYCLES,
                    DrivetrainAutoDatalogConfig.TOTAL_CYCLES * 2,
                    DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS * DrivetrainAutoDatalogConfig.TOTAL_CYCLES * 2.0
                )
                telemetry.addData("Min/Max Power", "%.2f / %.2f", DrivetrainAutoDatalogConfig.MIN_DRIVE_POWER, DrivetrainAutoDatalogConfig.MAX_DRIVE_POWER)
                telemetry.addData("Encoders", "L=%d, R=%d", getCurrentLeftTicks(), getCurrentRightTicks())
                if (savedFileName != null) {
                    telemetry.addLine("Previous Log: $savedFileName ($savedRowCount rows)")
                }
            }

            TestPhase.FORWARD_RUN, TestPhase.BACKWARD_RUN -> {
                val dir = if (phase == TestPhase.FORWARD_RUN) "FORWARD >>" else "<< BACKWARD"
                telemetry.addLine("[RUNNING] $dir")
                telemetry.addData("Cycle", "%d / %d  |  Segment: %d / %d",
                    currentCycle, DrivetrainAutoDatalogConfig.TOTAL_CYCLES,
                    segmentIndex, DrivetrainAutoDatalogConfig.TOTAL_CYCLES * 2
                )
                telemetry.addData("Segment Progress", "%.2f / %.2f m (%.1f%%) | Rem: %.2f m",
                    segmentProgressMeters,
                    DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS,
                    (segmentProgressMeters / DrivetrainAutoDatalogConfig.TARGET_DISTANCE_METERS) * 100.0,
                    segmentRemainingMeters
                )
                telemetry.addData("Total Distance", "%.2f m | Time: %.1fs",
                    cumulativeDistanceMeters + segmentProgressMeters,
                    totalTestTimer.seconds()
                )
                if (drive != null) {
                    telemetry.addData("Motors", "L_Pwr=%.2f R_Pwr=%.2f | L_Ticks=%d R_Ticks=%d",
                        drive.leftMotorPower, drive.rightMotorPower,
                        drive.leftEncoderPosition, drive.rightEncoderPosition
                    )
                    telemetry.addData("Heading", "Now=%.1f deg Target=%.1f deg Error=%+.2f deg Correction=%+.3f",
                        drive.heading, drive.targetHeading, drive.headingError, drive.headingCorrection
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
                    telemetry.addData("Velocity", "L=%.0f R=%.0f ticks/s | Speed=%.2f m/s",
                        drive.leftActualVelocity, drive.rightActualVelocity,
                        drive.linearSpeedMmPerSecond / 1000.0
                    )
                    telemetry.addData("Current", "L=%.2fA R=%.2fA (Total=%.2fA)",
                        drive.leftCurrentAmps, drive.rightCurrentAmps,
                        drive.leftCurrentAmps + drive.rightCurrentAmps
                    )
                }
                telemetry.addData("Datalog", "[REC] (%d rows) -> %s", datalogger.rowCount, datalogger.fileName ?: "logging...")
                telemetry.addLine("Press [B] to Abort")
            }

            TestPhase.FORWARD_SETTLE, TestPhase.BACKWARD_SETTLE -> {
                telemetry.addLine("[SETTLING] Pausing %.1fs between segments...".format(DrivetrainAutoDatalogConfig.SETTLE_TIME_SECONDS))
                telemetry.addData("Cycle", "%d / %d (Segment %d finished)",
                    currentCycle, DrivetrainAutoDatalogConfig.TOTAL_CYCLES, segmentIndex
                )
                telemetry.addData("Total Distance", "%.2f m", cumulativeDistanceMeters)
            }

            TestPhase.COMPLETED -> {
                telemetry.addLine("[COMPLETE] All 20 Segments Finished Successfully!")
                telemetry.addData("Total Distance", "%.2f meters", cumulativeDistanceMeters)
                telemetry.addData("Total Test Time", "%.2f seconds", totalTestTimer.seconds())
                telemetry.addData("Datalog Saved", "%s (%d rows)", savedFileName ?: "saved", savedRowCount)
                telemetry.addLine("File saved in /sdcard/FIRST/Datalogs/ (Retrieve via Web UI / ADB)")
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
        if (datalogger.isLogging) {
            savedRowCount = datalogger.rowCount
            savedFileName = datalogger.fileName
            datalogger.stopLogging()
        }
    }
}
