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
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

@Config
object DrivetrainAutoDatalogConfig {
    @JvmField var AUTO_START_ON_PLAY = true
    @JvmField var TOTAL_MASTER_CYCLES = 1 // 1 Master Cycle runs all enabled sub-profiles
    @JvmField var DEFAULT_TARGET_DISTANCE_METERS = 3.0
    @JvmField var RAPID_REVERSAL_DISTANCE_METERS = 2.5
    @JvmField var DEFAULT_SETTLE_TIME_SECONDS = 0.40
    @JvmField var SEGMENT_TIMEOUT_SECONDS = 15.0

    // Multi-Profile Test Suite Toggles
    @JvmField var RUN_SPEED_SWEEP_5 = true       // 5 sub-cycles: 0.30, 0.45, 0.60, 0.75, 1.00
    @JvmField var RUN_RAPID_REVERSAL = true      // 100% Fwd snap to 100% Rev (joystick slam stability)
    @JvmField var RUN_STEP_ACCEL = true          // 0 -> 100% instant kick (wheel slip & stall current)
    @JvmField var RUN_DISTURBANCE_PULSE = true   // Mid-course turn bump (active heading recovery)

    // Matches TeleOp: FTC stick-forward is negative, and negative drive power is physical forward.
    @JvmField var FORWARD_DIRECTION_SIGN = -1.0
    @JvmField var ENABLE_MANUAL_DRIVE_WHEN_IDLE = true
}

/**
 * Advanced Multi-Profile Drivetrain Autonomous Datalogger.
 *
 * Master Cycle contains rich sub-cycles:
 * 1. 5-Speed Sweeps (0.30, 0.45, 0.60, 0.75, 1.00) Forward & Backward 3.0m
 * 2. Rapid Reversal Snap (Full 100% Fwd -> Instant 100% Rev without pause)
 * 3. Step Acceleration Kick (0 -> 100% step command)
 * 4. Mid-course Heading Disturbance Bump & Active Recovery
 */
@Autonomous(name = "Drivetrain Auto Datalog (Multi-Profile 3m)", group = "Autonomous")
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

    enum class ProfileType {
        SPEED_SWEEP,
        RAPID_REVERSAL,
        STEP_ACCEL,
        DISTURBANCE_PULSE
    }

    data class SubProfile(
        val subIndex: Int,
        val name: String,
        val description: String,
        val type: ProfileType,
        val targetDistanceM: Double = 3.0,
        val maxPower: Double = 0.70,
        val minPower: Double = 0.22,
        val rampUpDistanceM: Double = 0.35,
        val rampDownDistanceM: Double = 0.75,
        val settleTimeSec: Double = 0.40
    )

    private lateinit var drivetrain: Drivetrain
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var dashboard: FtcDashboard

    private val datalogger = DrivetrainDatalogger()

    private var subProfiles: List<SubProfile> = emptyList()
    private var currentSubProfileIndex = 0 // 0 until subProfiles.size
    private var currentMasterCycle = 1 // 1..TOTAL_MASTER_CYCLES
    private var globalSegmentIndex = 0 // 1..totalSegments

    private var phase = TestPhase.IDLE
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

    private fun buildSubProfiles(): List<SubProfile> {
        val list = mutableListOf<SubProfile>()
        var idx = 1
        if (DrivetrainAutoDatalogConfig.RUN_SPEED_SWEEP_5) {
            list.add(SubProfile(idx++, "Speed_0.38_LowCruise", "Speed 0.38 (Static Friction & Low Speed Drift)", ProfileType.SPEED_SWEEP, 3.0, 0.38, 0.28, 0.25, 0.55))
            list.add(SubProfile(idx++, "Speed_0.50_LowMid", "Speed 0.50 (Low-Mid Linearity)", ProfileType.SPEED_SWEEP, 3.0, 0.50, 0.28, 0.30, 0.60))
            list.add(SubProfile(idx++, "Speed_0.65_Cruise", "Speed 0.65 (Standard Auto Cruise)", ProfileType.SPEED_SWEEP, 3.0, 0.65, 0.28, 0.35, 0.70))
            list.add(SubProfile(idx++, "Speed_0.80_HighSpeed", "Speed 0.80 (High Speed Sprint)", ProfileType.SPEED_SWEEP, 3.0, 0.80, 0.30, 0.35, 0.75))
            list.add(SubProfile(idx++, "Speed_1.00_MaxTurbo", "Speed 1.00 (Max Turbo / Velocity Saturation)", ProfileType.SPEED_SWEEP, 3.0, 1.00, 0.30, 0.40, 0.85))
        }
        if (DrivetrainAutoDatalogConfig.RUN_RAPID_REVERSAL) {
            list.add(SubProfile(idx++, "Rapid_Reversal_Snap", "Rapid Reversal Snap (100% Fwd -> 100% Rev Slam)", ProfileType.RAPID_REVERSAL, DrivetrainAutoDatalogConfig.RAPID_REVERSAL_DISTANCE_METERS, 1.00, 1.00, 0.0, 0.0, settleTimeSec = 0.50))
        }
        if (DrivetrainAutoDatalogConfig.RUN_STEP_ACCEL) {
            list.add(SubProfile(idx++, "Step_Acceleration", "Step Acceleration 0->100% (Wheel Slip & Stall Current)", ProfileType.STEP_ACCEL, 3.0, 1.00, 0.22, 0.0, 0.85))
        }
        if (DrivetrainAutoDatalogConfig.RUN_DISTURBANCE_PULSE) {
            list.add(SubProfile(idx++, "Heading_Bump_Recovery", "Heading Bump Disturbance Recovery", ProfileType.DISTURBANCE_PULSE, 3.0, 0.65, 0.22, 0.35, 0.75))
        }

        if (list.isEmpty()) {
            list.add(SubProfile(1, "Speed_0.70_Default", "Default 0.70 Profile", ProfileType.SPEED_SWEEP, 3.0, 0.70, 0.22, 0.35, 0.75))
        }
        return list
    }

    override fun init() {
        dashboard = FtcDashboard.getInstance()
        drivetrain = Drivetrain(hardwareMap)
        voltageSensor = hardwareMap.voltageSensor.iterator().next()

        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        for (module in lynxModules) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
        }

        // Configure telemetry flags: Full electrical/dynamics telemetry, Vision disabled for drivetrain
        DrivetrainConfig.DATALOG_ENABLED = true
        RobotConfig.DATALOG_ENABLED = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_TUNING = true
        DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION = true
        DrivetrainConfig.ENABLE_CURRENT_TELEMETRY = true
        DrivetrainConfig.ENABLE_VISION_TELEMETRY = false
        DrivetrainConfig.ENABLE_EXTENDED_IMU_TELEMETRY = true

        subProfiles = buildSubProfiles()
        currentSubProfileIndex = 0
        currentMasterCycle = 1
        globalSegmentIndex = 0
        phase = TestPhase.IDLE
        cumulativeDistanceMeters = 0.0
        abortReason = null
        savedFileName = null
        savedRowCount = 0

        val totalSegments = subProfiles.size * 2 * DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES
        telemetry.addLine("Multi-Profile Drivetrain Datalogger Initialized")
        telemetry.addData("Sub-Profiles", "%d profiles per Master Cycle (%d total passes)", subProfiles.size, totalSegments)
        for (p in subProfiles) {
            telemetry.addData("  [#${p.subIndex}]", "%s (Pwr=%.2f, %.1fm)", p.name, p.maxPower, p.targetDistanceM)
        }
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
        lynxModules.forEach { it.clearBulkCache() }
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

        val profile = subProfiles.getOrNull(currentSubProfileIndex) ?: subProfiles.first()

        // State Machine Execution
        var commandedForward = 0.0
        var commandedTurn = 0.0

        when (phase) {
            TestPhase.IDLE -> {
                if (DrivetrainAutoDatalogConfig.ENABLE_MANUAL_DRIVE_WHEN_IDLE) {
                    commandedForward = gamepad1.left_stick_y.toDouble()
                    commandedTurn = gamepad1.right_stick_x.toDouble()
                }
            }

            TestPhase.FORWARD_RUN -> {
                val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
                val leftDeltaM = abs(segmentStartLeftTicks - getCurrentLeftTicks()) * (mmPerTick / 1000.0)
                val rightDeltaM = abs(segmentStartRightTicks - getCurrentRightTicks()) * (mmPerTick / 1000.0)
                segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0
                segmentRemainingMeters = max(0.0, profile.targetDistanceM - segmentProgressMeters)

                // Mid-course turn disturbance injection for disturbance profile
                if (profile.type == ProfileType.DISTURBANCE_PULSE && segmentProgressMeters in 1.2..1.5) {
                    commandedTurn = 0.25 // Simulated bump pulse
                }

                // Segment Watchdog
                val segmentTimeoutSec = max(DrivetrainAutoDatalogConfig.SEGMENT_TIMEOUT_SECONDS, (profile.targetDistanceM / (profile.maxPower * 0.6)) + 5.0)
                if (segmentTimer.seconds() >= segmentTimeoutSec) {
                    abortTest("Forward segment timed out (%.1fs > %.1fs) in profile %s"
                        .format(segmentTimer.seconds(), segmentTimeoutSec, profile.name))
                } else if (segmentProgressMeters >= profile.targetDistanceM) {
                    cumulativeDistanceMeters += segmentProgressMeters

                    if (profile.type == ProfileType.RAPID_REVERSAL) {
                        // Rapid Reversal Snap: Immediately reverse without settling!
                        phase = TestPhase.BACKWARD_RUN
                        globalSegmentIndex++
                        recordSegmentStartTicks(profile.targetDistanceM)
                        segmentTimer.reset()
                    } else {
                        phase = TestPhase.FORWARD_SETTLE
                        settleTimer.reset()
                        commandedForward = 0.0
                    }
                } else {
                    commandedForward = calculateProfilePower(
                        progress = segmentProgressMeters,
                        remaining = segmentRemainingMeters,
                        profile = profile,
                        directionSign = 1.0
                    )
                }
            }

            TestPhase.FORWARD_SETTLE -> {
                commandedForward = 0.0
                if (settleTimer.seconds() >= profile.settleTimeSec) {
                    phase = TestPhase.BACKWARD_RUN
                    globalSegmentIndex++
                    recordSegmentStartTicks(profile.targetDistanceM)
                    segmentTimer.reset()
                }
            }

            TestPhase.BACKWARD_RUN -> {
                val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
                val leftDeltaM = abs(segmentStartLeftTicks - getCurrentLeftTicks()) * (mmPerTick / 1000.0)
                val rightDeltaM = abs(segmentStartRightTicks - getCurrentRightTicks()) * (mmPerTick / 1000.0)
                segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0
                segmentRemainingMeters = max(0.0, profile.targetDistanceM - segmentProgressMeters)

                // Segment Watchdog
                val backwardTimeoutSec = max(DrivetrainAutoDatalogConfig.SEGMENT_TIMEOUT_SECONDS, (profile.targetDistanceM / (profile.maxPower * 0.6)) + 5.0)
                if (segmentTimer.seconds() >= backwardTimeoutSec) {
                    abortTest("Backward segment timed out (%.1fs > %.1fs) in profile %s"
                        .format(segmentTimer.seconds(), backwardTimeoutSec, profile.name))
                } else if (segmentProgressMeters >= profile.targetDistanceM) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.BACKWARD_SETTLE
                    settleTimer.reset()
                    commandedForward = 0.0
                } else {
                    commandedForward = calculateProfilePower(
                        progress = segmentProgressMeters,
                        remaining = segmentRemainingMeters,
                        profile = profile,
                        directionSign = -1.0
                    )
                }
            }

            TestPhase.BACKWARD_SETTLE -> {
                commandedForward = 0.0
                if (settleTimer.seconds() >= profile.settleTimeSec) {
                    // Check if there are more sub-profiles in the current master cycle
                    if (currentSubProfileIndex + 1 < subProfiles.size) {
                        currentSubProfileIndex++
                        globalSegmentIndex++
                        val nextProfile = subProfiles[currentSubProfileIndex]
                        phase = TestPhase.FORWARD_RUN
                        recordSegmentStartTicks(nextProfile.targetDistanceM)
                        segmentTimer.reset()
                    } else if (currentMasterCycle < DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES) {
                        // Advance to next master cycle
                        currentMasterCycle++
                        currentSubProfileIndex = 0
                        globalSegmentIndex++
                        val nextProfile = subProfiles[0]
                        phase = TestPhase.FORWARD_RUN
                        recordSegmentStartTicks(nextProfile.targetDistanceM)
                        segmentTimer.reset()
                    } else {
                        // All cycles & profiles finished successfully!
                        completeTest()
                    }
                }
            }

            TestPhase.COMPLETED, TestPhase.ABORTED -> {
                commandedForward = 0.0
                commandedTurn = 0.0
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
                TestPhase.FORWARD_RUN -> "FORWARD"
                TestPhase.BACKWARD_RUN -> "BACKWARD"
                TestPhase.FORWARD_SETTLE, TestPhase.BACKWARD_SETTLE -> "SETTLE"
                TestPhase.COMPLETED -> "COMPLETE"
                TestPhase.ABORTED -> "ABORT"
                else -> "IDLE"
            }

            datalogger.writeRow(
                testPhase = "C${currentMasterCycle}_P${profile.subIndex}_${profile.name}_${directionLabel}",
                cycleIndex = currentMasterCycle,
                totalCycles = DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES,
                segmentIndex = globalSegmentIndex,
                segmentDirection = directionLabel,
                segmentTargetDistanceM = profile.targetDistanceM,
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
        packet.put("Master Cycle", "$currentMasterCycle/${DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES}")
        packet.put("Sub Profile", "[${profile.subIndex}/${subProfiles.size}] ${profile.name}")
        packet.put("Segment", "$globalSegmentIndex/${subProfiles.size * 2 * DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES}")
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
        }

        if (driveTelemetry != null) {
            packet.put("Heading (deg)", driveTelemetry.heading)
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
        renderTelemetry(driveTelemetry, profile)
    }

    private fun calculateProfilePower(
        progress: Double,
        remaining: Double,
        profile: SubProfile,
        directionSign: Double
    ): Double {
        if (profile.type == ProfileType.RAPID_REVERSAL) {
            return directionSign * DrivetrainAutoDatalogConfig.FORWARD_DIRECTION_SIGN * 1.0
        }

        val minP = profile.minPower
        val maxP = profile.maxPower
        val rampUp = profile.rampUpDistanceM
        val rampDown = profile.rampDownDistanceM

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
        subProfiles = buildSubProfiles()
        currentMasterCycle = 1
        currentSubProfileIndex = 0
        globalSegmentIndex = 1
        phase = TestPhase.FORWARD_RUN
        cumulativeDistanceMeters = 0.0
        abortReason = null

        val firstProfile = subProfiles[0]
        drivetrain.resetHeading(0.0)
        recordSegmentStartTicks(firstProfile.targetDistanceM)
        segmentTimer.reset()
        totalTestTimer.reset()

        datalogger.startLogging(prefix = "drivetrain_multiprofile_test")
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

    private fun recordSegmentStartTicks(targetDistM: Double) {
        segmentStartLeftTicks = drivetrain.getLeftEncoderPosition()
        segmentStartRightTicks = drivetrain.getRightEncoderPosition()
        segmentProgressMeters = 0.0
        segmentRemainingMeters = targetDistM
    }

    private fun getCurrentLeftTicks(): Int = drivetrain.getLeftEncoderPosition()

    private fun getCurrentRightTicks(): Int = drivetrain.getRightEncoderPosition()

    private fun renderTelemetry(drive: fgc.vietnam.robot01.Hardware.DriveTelemetry?, profile: SubProfile) {
        val totalSegments = subProfiles.size * 2 * DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES
        when (phase) {
            TestPhase.IDLE -> {
                telemetry.addLine("[READY] Multi-Profile Datalog Suite")
                telemetry.addData("Plan", "%d Master Cycles x %d Profiles = %d Total Passes",
                    DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES, subProfiles.size, totalSegments
                )
                telemetry.addData("Profiles", subProfiles.joinToString(" | ") { it.name })
                telemetry.addData("Encoders", "L=%d, R=%d", getCurrentLeftTicks(), getCurrentRightTicks())
                if (savedFileName != null) {
                    telemetry.addLine("Previous Log: $savedFileName ($savedRowCount rows)")
                }
            }

            TestPhase.FORWARD_RUN, TestPhase.BACKWARD_RUN -> {
                val dir = if (phase == TestPhase.FORWARD_RUN) "FORWARD >>" else "<< BACKWARD"
                telemetry.addLine("[RUNNING] $dir")
                telemetry.addData("Profile", "[%d/%d] %s (Target=%.2f Pwr)",
                    profile.subIndex, subProfiles.size, profile.name, profile.maxPower
                )
                telemetry.addData("Pass", "%d / %d (Cycle %d/%d)",
                    globalSegmentIndex, totalSegments, currentMasterCycle, DrivetrainAutoDatalogConfig.TOTAL_MASTER_CYCLES
                )
                telemetry.addData("Segment Progress", "%.2f / %.2f m (%.1f%%)",
                    segmentProgressMeters, profile.targetDistanceM,
                    (segmentProgressMeters / profile.targetDistanceM) * 100.0
                )
                telemetry.addData("Total Distance", "%.2f m | Time: %.1fs",
                    cumulativeDistanceMeters + segmentProgressMeters, totalTestTimer.seconds()
                )
                if (drive != null) {
                    telemetry.addData("Speed & Current", "%.2f m/s | L=%.1fA, R=%.1fA (Total=%.1fA)",
                        drive.linearSpeedMmPerSecond / 1000.0,
                        drive.leftCurrentAmps, drive.rightCurrentAmps,
                        drive.leftCurrentAmps + drive.rightCurrentAmps
                    )
                    telemetry.addData("Heading", "Now=%.1f deg Target=%.1f deg Error=%+.2f deg",
                        drive.heading, drive.targetHeading, drive.headingError
                    )
                }
                telemetry.addData("Datalog", "[REC] (%d rows) -> %s", datalogger.rowCount, datalogger.fileName ?: "logging...")
                telemetry.addLine("Press [B] to Abort")
            }

            TestPhase.FORWARD_SETTLE, TestPhase.BACKWARD_SETTLE -> {
                telemetry.addLine("[SETTLING] Pausing %.2fs between sub-profiles...".format(profile.settleTimeSec))
                telemetry.addData("Finished", "[%d/%d] %s", profile.subIndex, subProfiles.size, profile.name)
                telemetry.addData("Total Distance", "%.2f m", cumulativeDistanceMeters)
            }

            TestPhase.COMPLETED -> {
                telemetry.addLine("[COMPLETE] All %d Sub-Profiles Finished Successfully!".format(totalSegments))
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
