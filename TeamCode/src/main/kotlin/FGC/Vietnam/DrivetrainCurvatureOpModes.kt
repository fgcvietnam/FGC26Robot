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
import fgc.vietnam.robot01.Hardware.DriveTelemetry
import fgc.vietnam.robot01.Hardware.Drivetrain
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.concurrent.TimeUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.sin

@Config
object DrivetrainCurvatureConfig {
    @JvmField var AUTO_START_ON_PLAY = true

    // Slalom parameters (2.4m length x 0.7m width)
    @JvmField var SLALOM_DISTANCE_METERS = 2.40
    @JvmField var SLALOM_WAVELENGTH_METERS = 1.20 // 2 full sine waves
    @JvmField var SLALOM_MAX_STEER = 0.45

    // Spiral parameters (2.2m diameter circle)
    @JvmField var SPIRAL_MIN_RADIUS_M = 0.35
    @JvmField var SPIRAL_MAX_RADIUS_M = 1.10
    @JvmField var SPIRAL_EXPAND_TURNS = 1.25

    // J-Turn parameters (1.3m sprint -> 180° hook)
    @JvmField var J_TURN_SPRINT_DISTANCE_M = 1.30
    @JvmField var J_TURN_HOOK_POWER = 0.75
    @JvmField var J_TURN_HOOK_STEER = 0.85

    @JvmField var DEFAULT_SETTLE_TIME_SECONDS = 0.35
    @JvmField var SEGMENT_TIMEOUT_SECONDS = 20.0
    @JvmField var FORWARD_DIRECTION_SIGN = -1.0
    @JvmField var ENABLE_MANUAL_DRIVE_WHEN_IDLE = true
}

enum class CurvatureManeuverType {
    SLALOM,
    SPIRAL_LEFT,
    SPIRAL_RIGHT,
    J_TURN_LEFT,
    J_TURN_RIGHT
}

/**
 * Base Engine for Curvature Autonomous OpModes.
 */
abstract class BaseCurvatureDatalogOpMode(
    val testName: String,
    val maneuverType: CurvatureManeuverType,
    val forwardPower: Double,
    val logPrefix: String,
    val placementGuide: String
) : OpMode() {

    enum class TestPhase {
        IDLE,
        RUNNING_FORWARD,
        RUNNING_BACKWARD,
        RUNNING_EXPAND,
        RUNNING_CONTRACT,
        SPRINTING,
        HOOKING,
        SETTLING,
        COMPLETED,
        ABORTED
    }

    private lateinit var drivetrain: Drivetrain
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var dashboard: FtcDashboard

    private val datalogger = DrivetrainDatalogger()

    private var phase = TestPhase.IDLE
    private var cumulativeDistanceMeters = 0.0
    private var globalSegmentIndex = 1

    private var segmentStartLeftTicks = 0
    private var segmentStartRightTicks = 0
    private var segmentStartHeadingRad = 0.0
    private var segmentProgressMeters = 0.0
    private var segmentAccumulatedAngleRad = 0.0
    private var lastLoopHeadingRad = 0.0

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
        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        voltageSensor = hardwareMap.voltageSensor.firstOrNull()
            ?: error("No voltage sensor found in HardwareMap")

        drivetrain = Drivetrain(hardwareMap)
        drivetrain.resetHeading(0.0)

        globalSegmentIndex = 1
        phase = TestPhase.IDLE
        cumulativeDistanceMeters = 0.0
        abortReason = null

        telemetry.addLine("=== $testName ===")
        telemetry.addData("VỊ TRÍ ĐẶT BOT", placementGuide)
        telemetry.addData("Nút A / Cross", "Bắt đầu bài test")
        telemetry.addData("Nút B / Circle", "Dừng khẩn cấp (Abort)")
        telemetry.addData("Tự động Start khi Play", DrivetrainCurvatureConfig.AUTO_START_ON_PLAY)
        telemetry.update()
    }

    override fun start() {
        if (DrivetrainCurvatureConfig.AUTO_START_ON_PLAY && phase == TestPhase.IDLE) {
            beginTest()
        }
    }

    override fun loop() {
        lynxModules.forEach { it.clearBulkCache() }

        val batteryVoltage = voltageSensor.voltage
        val currentHeadingRad = drivetrain.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)

        // Query Hub Temperatures periodically (1000ms)
        val nowMs = System.currentTimeMillis()
        if (nowMs - lastTempQueryTimeMs >= 1000L) {
            lastTempQueryTimeMs = nowMs
            cachedHubTemps = lynxModules.map {
                try {
                    it.getTemperature(TempUnit.CELSIUS)
                } catch (_: Exception) {
                    Double.NaN
                }
            }
        }

        // Gamepad Buttons
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

        if (phase !in setOf(TestPhase.IDLE, TestPhase.COMPLETED, TestPhase.ABORTED)) {
            // Track wheel progress
            val mmPerTick = DrivetrainConfig.millimetersPerEncoderTick
            val leftDeltaM = abs(segmentStartLeftTicks - drivetrain.getLeftEncoderPosition()) * (mmPerTick / 1000.0)
            val rightDeltaM = abs(segmentStartRightTicks - drivetrain.getRightEncoderPosition()) * (mmPerTick / 1000.0)
            segmentProgressMeters = (leftDeltaM + rightDeltaM) / 2.0

            // Track continuous accumulated rotation
            var dAngle = currentHeadingRad - lastLoopHeadingRad
            while (dAngle > PI) dAngle -= 2 * PI
            while (dAngle < -PI) dAngle += 2 * PI
            segmentAccumulatedAngleRad += dAngle
            lastLoopHeadingRad = currentHeadingRad

            // Global Watchdog
            if (segmentTimer.seconds() >= DrivetrainCurvatureConfig.SEGMENT_TIMEOUT_SECONDS && phase != TestPhase.SETTLING) {
                abortTest("Segment timed out (%.1fs > %.1fs)"
                    .format(segmentTimer.seconds(), DrivetrainCurvatureConfig.SEGMENT_TIMEOUT_SECONDS))
            }
        }

        // State Machine Execution
        when (phase) {
            TestPhase.IDLE -> {
                if (DrivetrainCurvatureConfig.ENABLE_MANUAL_DRIVE_WHEN_IDLE) {
                    commandedForward = gamepad1.left_stick_y.toDouble()
                    commandedTurn = gamepad1.right_stick_x.toDouble()
                }
            }

            // === 1. SLALOM S-CURVE STATES ===
            TestPhase.RUNNING_FORWARD -> {
                val targetDist = DrivetrainCurvatureConfig.SLALOM_DISTANCE_METERS
                if (segmentProgressMeters >= targetDist) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.SETTLING
                    settleTimer.reset()
                    commandedForward = 0.0
                    commandedTurn = 0.0
                } else {
                    val s = (segmentProgressMeters / targetDist).coerceIn(0.0, 1.0)
                    val speedNorm = 0.35 + (forwardPower - 0.35) * sin(PI * s)
                    commandedForward = DrivetrainCurvatureConfig.FORWARD_DIRECTION_SIGN * speedNorm

                    val wavePhase = (2.0 * PI / DrivetrainCurvatureConfig.SLALOM_WAVELENGTH_METERS) * segmentProgressMeters
                    commandedTurn = DrivetrainCurvatureConfig.SLALOM_MAX_STEER * sin(wavePhase)
                }
            }

            TestPhase.RUNNING_BACKWARD -> {
                val targetDist = DrivetrainCurvatureConfig.SLALOM_DISTANCE_METERS
                if (segmentProgressMeters >= targetDist) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    completeTest()
                } else {
                    val s = (segmentProgressMeters / targetDist).coerceIn(0.0, 1.0)
                    val speedNorm = 0.35 + (forwardPower - 0.35) * sin(PI * s)
                    commandedForward = -DrivetrainCurvatureConfig.FORWARD_DIRECTION_SIGN * speedNorm

                    val wavePhase = (2.0 * PI / DrivetrainCurvatureConfig.SLALOM_WAVELENGTH_METERS) * segmentProgressMeters
                    commandedTurn = -DrivetrainCurvatureConfig.SLALOM_MAX_STEER * sin(wavePhase)
                }
            }

            // === 2. ARCHIMEDES SPIRAL STATES ===
            TestPhase.RUNNING_EXPAND -> {
                val targetAngleRad = DrivetrainCurvatureConfig.SPIRAL_EXPAND_TURNS * 2.0 * PI
                val turnSign = if (maneuverType == CurvatureManeuverType.SPIRAL_LEFT) 1.0 else -1.0

                if (abs(segmentAccumulatedAngleRad) >= targetAngleRad) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.RUNNING_CONTRACT
                    recordSegmentStart()
                } else {
                    val progressNorm = (abs(segmentAccumulatedAngleRad) / targetAngleRad).coerceIn(0.0, 1.0)
                    val r = DrivetrainCurvatureConfig.SPIRAL_MIN_RADIUS_M +
                            (DrivetrainCurvatureConfig.SPIRAL_MAX_RADIUS_M - DrivetrainCurvatureConfig.SPIRAL_MIN_RADIUS_M) * progressNorm

                    val trackWidthM = DrivetrainConfig.TRACK_WIDTH_MM / 1000.0
                    val steerRatio = (trackWidthM / (2.0 * r)).coerceIn(0.15, 0.75)

                    commandedForward = DrivetrainCurvatureConfig.FORWARD_DIRECTION_SIGN * forwardPower
                    commandedTurn = turnSign * steerRatio
                }
            }

            TestPhase.RUNNING_CONTRACT -> {
                val targetAngleRad = DrivetrainCurvatureConfig.SPIRAL_EXPAND_TURNS * 2.0 * PI
                val turnSign = if (maneuverType == CurvatureManeuverType.SPIRAL_LEFT) 1.0 else -1.0

                if (abs(segmentAccumulatedAngleRad) >= targetAngleRad) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    completeTest()
                } else {
                    val progressNorm = (abs(segmentAccumulatedAngleRad) / targetAngleRad).coerceIn(0.0, 1.0)
                    val r = DrivetrainCurvatureConfig.SPIRAL_MAX_RADIUS_M -
                            (DrivetrainCurvatureConfig.SPIRAL_MAX_RADIUS_M - DrivetrainCurvatureConfig.SPIRAL_MIN_RADIUS_M) * progressNorm

                    val trackWidthM = DrivetrainConfig.TRACK_WIDTH_MM / 1000.0
                    val steerRatio = (trackWidthM / (2.0 * r)).coerceIn(0.15, 0.75)

                    commandedForward = DrivetrainCurvatureConfig.FORWARD_DIRECTION_SIGN * forwardPower
                    commandedTurn = turnSign * steerRatio
                }
            }

            // === 3. J-TURN 180° STATES ===
            TestPhase.SPRINTING -> {
                if (segmentProgressMeters >= DrivetrainCurvatureConfig.J_TURN_SPRINT_DISTANCE_M) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    phase = TestPhase.HOOKING
                    recordSegmentStart()
                } else {
                    commandedForward = DrivetrainCurvatureConfig.FORWARD_DIRECTION_SIGN * forwardPower
                    commandedTurn = 0.0
                }
            }

            TestPhase.HOOKING -> {
                val targetAngleRad = PI // 180 deg
                val turnSign = if (maneuverType == CurvatureManeuverType.J_TURN_LEFT) 1.0 else -1.0

                if (abs(segmentAccumulatedAngleRad) >= targetAngleRad * 0.95) {
                    cumulativeDistanceMeters += segmentProgressMeters
                    completeTest()
                } else {
                    commandedForward = DrivetrainCurvatureConfig.FORWARD_DIRECTION_SIGN * DrivetrainCurvatureConfig.J_TURN_HOOK_POWER
                    commandedTurn = turnSign * DrivetrainCurvatureConfig.J_TURN_HOOK_STEER
                }
            }

            TestPhase.SETTLING -> {
                commandedForward = 0.0
                commandedTurn = 0.0
                if (settleTimer.seconds() >= DrivetrainCurvatureConfig.DEFAULT_SETTLE_TIME_SECONDS) {
                    if (maneuverType == CurvatureManeuverType.SLALOM) {
                        phase = TestPhase.RUNNING_BACKWARD
                        globalSegmentIndex++
                        recordSegmentStart()
                    } else {
                        completeTest()
                    }
                }
            }

            TestPhase.COMPLETED, TestPhase.ABORTED -> {
                commandedForward = 0.0
                commandedTurn = 0.0
                drivetrain.stop()
            }
        }

        // Apply Motor Power
        val driveTelemetry = if (phase != TestPhase.IDLE) {
            drivetrain.driveDirect(
                forward = commandedForward,
                turn = commandedTurn,
                batteryVoltage = batteryVoltage,
                holdHeading = false
            )
        } else {
            drivetrain.drive(
                forward = commandedForward,
                turn = commandedTurn,
                precisionMode = false,
                batteryVoltage = batteryVoltage
            )
        }

        // Datalog Row Writing
        if (datalogger.isLogging && driveTelemetry != null) {
            val totalDistance = cumulativeDistanceMeters + segmentProgressMeters
            val phaseLabel = "${logPrefix}_${phase.name}"

            datalogger.writeRow(
                testPhase = phaseLabel,
                cycleIndex = 1,
                totalCycles = 1,
                segmentIndex = globalSegmentIndex,
                segmentDirection = phase.name,
                segmentTargetDistanceM = 3.0,
                segmentProgressM = segmentProgressMeters,
                segmentRemainingM = max(0.0, 3.0 - segmentProgressMeters),
                totalDistanceM = totalDistance,
                commandedForwardPwr = commandedForward,
                commandedTurnPwr = commandedTurn,
                drive = driveTelemetry,
                hubTemperaturesCelsius = cachedHubTemps
            )
        }

        // Dashboard & Driver Station Telemetry
        renderDashboard(driveTelemetry)
    }

    private fun recordSegmentStart() {
        segmentStartLeftTicks = drivetrain.getLeftEncoderPosition()
        segmentStartRightTicks = drivetrain.getRightEncoderPosition()
        segmentStartHeadingRad = drivetrain.imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        lastLoopHeadingRad = segmentStartHeadingRad
        segmentProgressMeters = 0.0
        segmentAccumulatedAngleRad = 0.0
        segmentTimer.reset()
    }

    private fun beginTest() {
        globalSegmentIndex = 1
        cumulativeDistanceMeters = 0.0
        abortReason = null

        datalogger.startLogging(prefix = logPrefix)
        totalTestTimer.reset()
        lynxModules.forEach { it.setPattern(runningPattern) }

        recordSegmentStart()
        phase = when (maneuverType) {
            CurvatureManeuverType.SLALOM -> TestPhase.RUNNING_FORWARD
            CurvatureManeuverType.SPIRAL_LEFT, CurvatureManeuverType.SPIRAL_RIGHT -> TestPhase.RUNNING_EXPAND
            CurvatureManeuverType.J_TURN_LEFT, CurvatureManeuverType.J_TURN_RIGHT -> TestPhase.SPRINTING
        }
    }

    private fun completeTest() {
        phase = TestPhase.COMPLETED
        drivetrain.stop()
        savedFileName = datalogger.fileName
        savedRowCount = datalogger.rowCount
        datalogger.stopLogging()
        lynxModules.forEach { it.setPattern(completePattern) }
    }

    private fun abortTest(reason: String) {
        phase = TestPhase.ABORTED
        abortReason = reason
        drivetrain.stop()
        savedFileName = datalogger.fileName
        savedRowCount = datalogger.rowCount
        datalogger.stopLogging()
        lynxModules.forEach { it.setPattern(abortPattern) }
    }

    private fun renderDashboard(driveTelemetry: DriveTelemetry?) {
        val packet = TelemetryPacket()

        packet.put("Test", testName)
        packet.put("Phase", phase.name)
        packet.put("Progress (m)", "%.3f".format(segmentProgressMeters))
        packet.put("Accumulated Angle (deg)", "%.1f".format(Math.toDegrees(segmentAccumulatedAngleRad)))
        packet.put("Elapsed (s)", "%.2f".format(totalTestTimer.seconds()))

        if (driveTelemetry != null) {
            packet.put("Speed (m/s)", "%.2f".format(driveTelemetry.linearSpeedMmPerSecond / 1000.0))
            packet.put("Yaw Rate (deg/s)", "%.1f".format(driveTelemetry.yawRate))
            packet.put("Total Current (A)", "%.2f".format(driveTelemetry.leftCurrentAmps + driveTelemetry.rightCurrentAmps))
            packet.put("Battery (V)", "%.2f".format(driveTelemetry.batteryVoltage))

            val poseInches = Pose2d(
                Vector2d(driveTelemetry.poseX / 25.4, driveTelemetry.poseY / 25.4),
                Math.toRadians(driveTelemetry.poseHeading)
            )
            drawRobot(packet.fieldOverlay(), poseInches)
        }

        packet.put("Datalog Rows", datalogger.rowCount.takeIf { it > 0 } ?: savedRowCount)
        dashboard.sendTelemetryPacket(packet)

        // Driver Station Telemetry
        telemetry.addLine("=== $testName ===")
        telemetry.addData("VỊ TRÍ ĐẶT BOT", placementGuide)
        telemetry.addData("Trạng thái", phase.name)
        telemetry.addData("Tiến độ", "%.2f m | %.1f°".format(segmentProgressMeters, Math.toDegrees(segmentAccumulatedAngleRad)))
        telemetry.addData("Datalog Rows", datalogger.rowCount.takeIf { it > 0 } ?: savedRowCount)
        if (savedFileName != null) {
            telemetry.addData("Saved CSV", savedFileName)
        }
        if (abortReason != null) {
            telemetry.addData("Lý do Abort", abortReason)
        }
        telemetry.update()
    }

    override fun stop() {
        if (datalogger.isLogging) {
            datalogger.stopLogging()
        }
        drivetrain.stop()
    }
}

// ==============================================================================
// 8 INDIVIDUAL AUTONOMOUS OPMODES REGISTERED IN DRIVER STATION DROPDOWN
// ==============================================================================

@Autonomous(name = "Datalog: 1. Slalom S-Curve (Med 0.65)", group = "Curvature Datalog")
class DrivetrainSlalomMedOpMode : BaseCurvatureDatalogOpMode(
    testName = "1. Slalom S-Curve (Medium Speed 0.65)",
    maneuverType = CurvatureManeuverType.SLALOM,
    forwardPower = 0.65,
    logPrefix = "drivetrain_slalom_med",
    placementGuide = "Sát mép sau, chính giữa theo chiều ngang (X=0, Y=-1.2m), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 2. Slalom S-Curve (Max 1.00)", group = "Curvature Datalog")
class DrivetrainSlalomMaxOpMode : BaseCurvatureDatalogOpMode(
    testName = "2. Slalom S-Curve (Max Speed 1.00)",
    maneuverType = CurvatureManeuverType.SLALOM,
    forwardPower = 1.00,
    logPrefix = "drivetrain_slalom_max",
    placementGuide = "Sát mép sau, chính giữa theo chiều ngang (X=0, Y=-1.2m), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 3. Spiral Left CCW (Med 0.60)", group = "Curvature Datalog")
class DrivetrainSpiralLeftMedOpMode : BaseCurvatureDatalogOpMode(
    testName = "3. Archimedes Spiral Left CCW (Medium Speed 0.60)",
    maneuverType = CurvatureManeuverType.SPIRAL_LEFT,
    forwardPower = 0.60,
    logPrefix = "drivetrain_spiral_left_med",
    placementGuide = "CHÍNH GIỮA TÂM SÂN (X=0, Y=0), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 4. Spiral Left CCW (Max 0.95)", group = "Curvature Datalog")
class DrivetrainSpiralLeftMaxOpMode : BaseCurvatureDatalogOpMode(
    testName = "4. Archimedes Spiral Left CCW (Max Speed 0.95)",
    maneuverType = CurvatureManeuverType.SPIRAL_LEFT,
    forwardPower = 0.95,
    logPrefix = "drivetrain_spiral_left_max",
    placementGuide = "CHÍNH GIỮA TÂM SÂN (X=0, Y=0), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 5. Spiral Right CW (Med 0.60)", group = "Curvature Datalog")
class DrivetrainSpiralRightMedOpMode : BaseCurvatureDatalogOpMode(
    testName = "5. Archimedes Spiral Right CW (Medium Speed 0.60)",
    maneuverType = CurvatureManeuverType.SPIRAL_RIGHT,
    forwardPower = 0.60,
    logPrefix = "drivetrain_spiral_right_med",
    placementGuide = "CHÍNH GIỮA TÂM SÂN (X=0, Y=0), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 6. Spiral Right CW (Max 0.95)", group = "Curvature Datalog")
class DrivetrainSpiralRightMaxOpMode : BaseCurvatureDatalogOpMode(
    testName = "6. Archimedes Spiral Right CW (Max Speed 0.95)",
    maneuverType = CurvatureManeuverType.SPIRAL_RIGHT,
    forwardPower = 0.95,
    logPrefix = "drivetrain_spiral_right_max",
    placementGuide = "CHÍNH GIỮA TÂM SÂN (X=0, Y=0), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 7. J-Turn 180° Left (Sprint+Hook)", group = "Curvature Datalog")
class DrivetrainJTurnLeftOpMode : BaseCurvatureDatalogOpMode(
    testName = "7. High-Speed J-Turn 180° Left (Sprint + Hook)",
    maneuverType = CurvatureManeuverType.J_TURN_LEFT,
    forwardPower = 1.00,
    logPrefix = "drivetrain_jturn_left",
    placementGuide = "Góc sau bên PHẢI (X=+0.5m, Y=-1.2m), mũi hướng thẳng về trước."
)

@Autonomous(name = "Datalog: 8. J-Turn 180° Right (Sprint+Hook)", group = "Curvature Datalog")
class DrivetrainJTurnRightOpMode : BaseCurvatureDatalogOpMode(
    testName = "8. High-Speed J-Turn 180° Right (Sprint + Hook)",
    maneuverType = CurvatureManeuverType.J_TURN_RIGHT,
    forwardPower = 1.00,
    logPrefix = "drivetrain_jturn_right",
    placementGuide = "Góc sau bên TRÁI (X=-0.5m, Y=-1.2m), mũi hướng thẳng về trước."
)
