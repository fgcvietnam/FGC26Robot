package FGC.Vietnam

import fgc.vietnam.robot01.Hardware.Climb
import fgc.vietnam.robot01.Hardware.ClimbTelemetry
import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.config.Config
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.Blinker
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import FGC.Vietnam.Config.ClimbConfig
import FGC.Vietnam.Config.DrivetrainConfig
import FGC.Vietnam.DataLogger.ClimbDatalogger
import fgc.vietnam.robot01.Hardware.Drivetrain
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.Locale
import java.util.concurrent.TimeUnit
import kotlin.math.abs
import kotlin.math.sqrt

@Config
object ClimbTeleOpDatalogConfig {
    @JvmField var AUTO_START_ON_PLAY = true
    @JvmField var ENABLE_DRIVETRAIN_ASSIST = true
    @JvmField var MIN_TRIGGER_THRESHOLD = 0.05
}

/**
 * Dedicated Human-Controlled Climb TeleOp Datalogging OpMode.
 *
 * Controls:
 * - Right Trigger: Climb Spool Up (Lift Robot)
 * - Left Trigger: Active Power Hold
 * - Touchpad: Climb Spool Down (Reverse/Release)
 * - Triangle: Extend Climb Hook
 * - Triangle + Touchpad: Retract Climb Hook
 * - Left Stick Y / Right Stick X: Optional Drivetrain Positioning
 *
 * Automatically records comprehensive 2-motor electrical/mechanical specs,
 * full 3D IMU orientation & angular rates, distance sensor, and driver inputs from START to STOP.
 */
@TeleOp(name = "Climb TeleOp Datalog (Human Controlled)", group = "Datalog")
class ClimbTeleOpDatalogOpMode : OpMode() {

    private lateinit var climb: Climb
    private lateinit var drivetrain: Drivetrain
    private lateinit var voltageSensor: VoltageSensor
    private lateinit var lynxModules: List<LynxModule>
    private lateinit var dashboard: FtcDashboard

    private val datalogger = ClimbDatalogger()
    private val loopTimer = ElapsedTime()
    private val sessionTimer = ElapsedTime()

    private var lastTempQueryTimeMs = 0L
    private var cachedHubTemps: List<Double> = emptyList()

    private var savedFileName: String? = null
    private var savedRowCount: Int = 0

    private val loggingPattern = listOf(
        Blinker.Step(Color.CYAN, 250, TimeUnit.MILLISECONDS),
        Blinker.Step(Color.BLUE, 250, TimeUnit.MILLISECONDS)
    )

    private val completePattern = listOf(
        Blinker.Step(Color.GREEN, 1, TimeUnit.SECONDS)
    )

    override fun init() {
        dashboard = FtcDashboard.getInstance()
        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        lynxModules.forEach { it.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL }

        voltageSensor = hardwareMap.voltageSensor.firstOrNull()
            ?: error("No voltage sensor found in HardwareMap")

        climb = Climb(hardwareMap)
        drivetrain = Drivetrain(hardwareMap)

        telemetry.addLine("=== CLIMB TELEOP DATALOGGER READY ===")
        telemetry.addLine("• Right Trigger : Climb Forward (Lift)")
        telemetry.addLine("• Left Trigger  : Climb Hold")
        telemetry.addLine("• Touchpad      : Climb Reverse (Release)")
        telemetry.addLine("• Triangle      : Extend Hook")
        telemetry.addLine("Datalogging will automatically start when PLAY is pressed.")
        telemetry.update()
    }

    override fun start() {
        sessionTimer.reset()
        loopTimer.reset()
        datalogger.startLogging(prefix = "climb_teleop")
        lynxModules.forEach { it.setPattern(loggingPattern) }
    }

    override fun loop() {
        lynxModules.forEach { it.clearBulkCache() }

        val loopTimeMs = loopTimer.milliseconds()
        loopTimer.reset()

        val batteryVoltage = voltageSensor.voltage

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

        // ── Human Driver Inputs ──
        val climbForwardInput = gamepad1.right_trigger
        val climbHoldInput = gamepad1.left_trigger

        if (climbForwardInput > ClimbTeleOpDatalogConfig.MIN_TRIGGER_THRESHOLD) {
            climb.climbForward(climbForwardInput)
        } else if (climbHoldInput > ClimbTeleOpDatalogConfig.MIN_TRIGGER_THRESHOLD) {
            climb.updateHoldPower(climbHoldInput)
        } else if (gamepad1.touchpad) {
            climb.climbBackward(1.0)
        } else {
            climb.stop()
        }

        if (gamepad1.triangle || gamepad2.triangle) {
            if (gamepad1.touchpad || gamepad2.touchpad) {
                climb.climbRetract()
            } else {
                climb.climbExtend()
            }
        } else {
            climb.climbExtendStop()
        }

        // Optional Drivetrain positioning
        if (ClimbTeleOpDatalogConfig.ENABLE_DRIVETRAIN_ASSIST) {
            val forward = gamepad1.left_stick_y.toDouble()
            val turn = gamepad1.right_stick_x.toDouble()
            if (abs(forward) > 0.05 || abs(turn) > 0.05) {
                drivetrain.drive(forward = forward, turn = turn, precisionMode = false, batteryVoltage = batteryVoltage)
            } else {
                drivetrain.stop()
            }
        }

        // ── 3D IMU Readings ──
        val angles = drivetrain.imu.robotYawPitchRollAngles
        val angularVel = drivetrain.imu.getRobotAngularVelocity(AngleUnit.DEGREES)

        val yawDeg = angles.getYaw(AngleUnit.DEGREES)
        val pitchDeg = angles.getPitch(AngleUnit.DEGREES)
        val rollDeg = angles.getRoll(AngleUnit.DEGREES)

        val yawRateDegS = angularVel.zRotationRate.toDouble()
        val pitchRateDegS = angularVel.xRotationRate.toDouble()
        val rollRateDegS = angularVel.yRotationRate.toDouble()

        val totalTiltDeg = sqrt(pitchDeg * pitchDeg + rollDeg * rollDeg)

        // ── Climb Telemetry Extraction ──
        val climbTel = climb.getTelemetry(batteryVoltage)

        // ── Datalog Row Writing ──
        if (datalogger.isLogging) {
            datalogger.writeRow(
                climb = climbTel,
                batteryVoltage = batteryVoltage,
                yawDeg = yawDeg,
                pitchDeg = pitchDeg,
                rollDeg = rollDeg,
                yawRateDegS = yawRateDegS,
                pitchRateDegS = pitchRateDegS,
                rollRateDegS = rollRateDegS,
                gamepad1 = gamepad1,
                gamepad2 = gamepad2,
                hubTemperaturesCelsius = cachedHubTemps
            )
        }

        // ── Live Dashboard Telemetry ──
        val packet = TelemetryPacket()
        packet.put("Climb State", climbTel.climbState)
        packet.put("Above Current (A)", climbTel.motorAboveCurrentAmps)
        packet.put("Below Current (A)", climbTel.motorBelowCurrentAmps)
        packet.put("Total Current (A)", climbTel.totalCurrentAmps)
        packet.put("Total Power (W)", climbTel.totalPowerWatts)
        packet.put("Motor Above RPM", climbTel.motorAboveRpm)
        packet.put("Motor Below RPM", climbTel.motorBelowRpm)
        packet.put("Shaft Above RPM", climbTel.shaftAboveRpm)
        packet.put("Shaft Below RPM", climbTel.shaftBelowRpm)
        packet.put("Encoder Diff (ticks)", climbTel.encoderPositionDiffTicks)
        packet.put("Pitch Tilt (deg)", pitchDeg)
        packet.put("Roll Tilt (deg)", rollDeg)
        packet.put("Total Tilt (deg)", totalTiltDeg)
        packet.put("Distance Sensor (mm)", climbTel.distanceMm)
        packet.put("Hold Power", climbTel.holdPower)
        packet.put("Battery (V)", batteryVoltage)
        packet.put("Datalog Rows", datalogger.rowCount.takeIf { it > 0 } ?: savedRowCount)
        dashboard.sendTelemetryPacket(packet)

        // ── Driver Station Telemetry ──
        telemetry.addLine("=== CLIMB TELEOP DATALOGGER [REC] ===")
        telemetry.addData("State", "%s | Hold: %.2f", climbTel.climbState, climbTel.holdPower)
        telemetry.addData("Currents", "Above: %.2f A | Below: %.2f A | Total: %.2f A",
            climbTel.motorAboveCurrentAmps, climbTel.motorBelowCurrentAmps, climbTel.totalCurrentAmps)
        telemetry.addData("Power", "%.1f Watts | Battery: %.2f V", climbTel.totalPowerWatts, batteryVoltage)
        telemetry.addData("Shaft RPM", "Above: %.1f | Below: %.1f | Diff: %d ticks",
            climbTel.shaftAboveRpm, climbTel.shaftBelowRpm, climbTel.encoderPositionDiffTicks)
        telemetry.addData("Robot Tilt", "Pitch: %.1f° | Roll: %.1f° | Total: %.1f°",
            pitchDeg, rollDeg, totalTiltDeg)
        telemetry.addData("Distance", if (climbTel.distanceMm.isNaN()) "N/A" else "%.1f mm".format(climbTel.distanceMm))
        telemetry.addData("Loop Time", "%.1f ms (%.1f Hz)", loopTimeMs, if (loopTimeMs > 0) 1000.0 / loopTimeMs else 0.0)
        telemetry.addData("Logged Rows", datalogger.rowCount.takeIf { it > 0 } ?: savedRowCount)
        if (savedFileName != null) {
            telemetry.addData("Saved CSV", savedFileName)
        }
        telemetry.update()
    }

    override fun stop() {
        climb.stop()
        drivetrain.stop()
        if (datalogger.isLogging) {
            savedFileName = datalogger.fileName
            savedRowCount = datalogger.rowCount
            datalogger.stopLogging()
        }
        lynxModules.forEach { it.setPattern(completePattern) }
    }
}
