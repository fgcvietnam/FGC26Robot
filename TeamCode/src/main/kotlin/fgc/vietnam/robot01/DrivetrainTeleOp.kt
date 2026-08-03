package fgc.vietnam.robot01

import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import kotlin.math.abs

enum class DriveControlMode {
    ARCADE,
    SPLIT_ARCADE,
    TANK,
}

abstract class SharedDriveTeleOp protected constructor(
    private val controlMode: DriveControlMode,
) : OpMode() {
    private lateinit var drivetrain: Drivetrain
    private lateinit var flywheel: Flywheel
    private lateinit var intake: Intake
    private lateinit var lynxModules: List<LynxModule>
    private val datalogger = TeleOpDatalogger()

    private var previousHeadingToggle = false
    private var previousFlywheelToggle = false
    private var previousHexToggle = false
    private var previousDatalogToggle = false

    override fun init() {
        drivetrain = Drivetrain(hardwareMap)
        flywheel = Flywheel(hardwareMap)
        intake = Intake(hardwareMap)
        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        telemetry.addData(
            "Status",
            if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
                "A heading hold | B flywheel | LT/RT Intake | X hex toggle | D-pad L/R Servos | Start+Y datalog"
            } else {
                "Tank: sticks Y | B flywheel | LT/RT Intake | X hex toggle | D-pad L/R Servos"
            },
        )
    }

    override fun loop() {
        val g1 = gamepad1
        val g2 = gamepad2

        val headingToggle = g1.a || g2.a
        if (
            (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) &&
            headingToggle &&
            !previousHeadingToggle
        ) {
            drivetrain.toggleHeadingHold()
        }
        previousHeadingToggle = headingToggle

        val flywheelToggle = g1.b || g2.b
        if (flywheelToggle && !previousFlywheelToggle) {
            flywheel.toggle()
        }
        previousFlywheelToggle = flywheelToggle

        val combinedRightStickY = -(g1.right_stick_y.toDouble() + g2.right_stick_y.toDouble())
        val combinedRightStickX = -(g1.right_stick_x.toDouble() + g2.right_stick_x.toDouble())
        val combinedLeftStickY = -(g1.left_stick_y.toDouble() + g2.left_stick_y.toDouble())

        val forwardInput = deadband(combinedRightStickY).coerceIn(-1.0, 1.0)
        val turnInput = deadband(combinedRightStickX).coerceIn(-1.0, 1.0)
        val tankLeft = deadband(combinedRightStickY).coerceIn(-1.0, 1.0)
        val tankRight = deadband(combinedLeftStickY).coerceIn(-1.0, 1.0)

        val drive = when (controlMode) {
            DriveControlMode.ARCADE -> drivetrain.drive(
                forward = forwardInput,
                turn = turnInput,
            )

            DriveControlMode.SPLIT_ARCADE -> drivetrain.drive(
                forward = deadband(combinedLeftStickY).coerceIn(-1.0, 1.0),
                turn = turnInput,
            )

            DriveControlMode.TANK -> drivetrain.driveTank(
                left = tankLeft,
                right = tankRight,
            )
        }
        val flywheelState = flywheel.update()

        val hexToggle = g1.x || g2.x
        if (hexToggle && !previousHexToggle) {
            intake.toggleHexDirection()
        }
        previousHexToggle = hexToggle

        // Start + Y  →  toggle full-robot datalog (Arcade / Split Arcade only)
        if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
            val datalogToggle = (g1.start && g1.y) || (g2.start && g2.y)
            if (datalogToggle && !previousDatalogToggle) {
                datalogger.toggleLogging()
            }
            previousDatalogToggle = datalogToggle

            if (datalogger.isLogging) {
                val hubTemps = lynxModules.map { hub ->
                    try { hub.getTemperature(TempUnit.CELSIUS) } catch (e: Exception) { Double.NaN }
                }
                datalogger.writeRow(
                    gamepad1 = gamepad1,
                    gamepad2 = gamepad2,
                    drive = drive,
                    flywheel = flywheelState,
                    intake = intake,
                    hubTemperaturesCelsius = hubTemps,
                )
            }
        }

        // Reversed Intake controls on Triggers (and Bumpers as fallback)
        // LT/LB = Intake, RT/RB = Outtake
        val intakeInput = g1.left_trigger > 0.1 || g2.left_trigger > 0.1 || g1.left_bumper || g2.left_bumper
        val outtakeInput = g1.right_trigger > 0.1 || g2.right_trigger > 0.1 || g1.right_bumper || g2.right_bumper

        if (intakeInput) {
            intake.startIntake()
        } else if (outtakeInput) {
            intake.startOuttake()
        } else {
            intake.stopMotor()
        }

        val servoBwd = g1.dpad_left || g2.dpad_left
        val servoFwd = g1.dpad_right || g2.dpad_right

        if (servoBwd) {
            intake.moveServosBackward()
        } else if (servoFwd) {
            intake.moveServosForward()
        } else {
            intake.stopServos()
        }

        when (controlMode) {
            DriveControlMode.ARCADE, DriveControlMode.SPLIT_ARCADE -> telemetry.addData(
                "Input",
                "forward=%.2f→%.2f turn=%.2f hold=%s",
                drive.requestedForward,
                drive.limitedForward,
                drive.requestedTurn,
                if (drive.headingHoldEnabled) "ON" else "OFF",
            )

            DriveControlMode.TANK -> telemetry.addData(
                "Input",
                "left motor(R stick)=%.2f right motor(L stick)=%.2f",
                tankLeft,
                tankRight,
            )
        }
        telemetry.addData(
            "Heading",
            "now=%.1f° target=%.1f° error=%+.2f° rate=%+.1f°/s",
            drive.heading,
            drive.targetHeading,
            drive.headingError,
            drive.yawRate,
        )
        telemetry.addData(
            "Heading PID",
            "P=%+.3f I=%+.3f D=%+.3f total=%+.3f",
            drive.proportionalCorrection,
            drive.integralCorrection,
            drive.derivativeCorrection,
            drive.headingCorrection,
        )
        telemetry.addData(
            "Velocity",
            "L=%.0f/%.0f R=%.0f/%.0f ticks/s",
            drive.leftActualVelocity,
            drive.leftTargetVelocity,
            drive.rightActualVelocity,
            drive.rightTargetVelocity,
        )
        telemetry.addData(
            "Drive current",
            "L=%.2f A R=%.2f A total=%.2f A",
            drive.leftCurrentAmps,
            drive.rightCurrentAmps,
            drive.leftCurrentAmps + drive.rightCurrentAmps,
        )
        telemetry.addData(
            "Robot",
            "speed=%.0f mm/s battery=%.2f V gear=%.1f:1",
            drive.linearSpeedMmPerSecond,
            drive.batteryVoltage,
            DrivetrainConfig.GEAR_REDUCTION,
        )
        telemetry.addData(
            "Flywheel",
            "%s %s shaft=%.0f/%.0f rpm difference=%.0f",
            if (flywheelState.enabled) "ON" else "OFF",
            if (flywheelState.atSpeed) "READY" else "—",
            flywheelState.shaftRpm,
            flywheelState.targetRpm,
            flywheelState.rpmDifference,
        )
        telemetry.addData(
            "Flywheel M2",
            "rpm=%.0f velocity=%.0f/%.0f ticks/s current=%.2f A",
            flywheelState.primaryMotor.rpm,
            flywheelState.primaryMotor.velocity,
            flywheelState.targetVelocity,
            flywheelState.primaryMotor.currentAmps,
        )
        telemetry.addData(
            "Flywheel M3",
            "rpm=%.0f velocity=%.0f/%.0f ticks/s current=%.2f A",
            flywheelState.secondaryMotor.rpm,
            flywheelState.secondaryMotor.velocity,
            flywheelState.targetVelocity,
            flywheelState.secondaryMotor.currentAmps,
        )
        telemetry.addData(
            "Intake",
            "hex=%s",
            if (intake.hexReversed) "OPPOSITE" else "SAME",
        )
        if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
            telemetry.addData(
                "Datalog",
                "%s rows=%d",
                if (datalogger.isLogging) "● REC" else "○ off",
                datalogger.rowCount,
            )
        }
        telemetry.addData(
            "Flywheel wheel",
            "rim=%.1f m/s",
            flywheelState.surfaceSpeedMetersPerSecond,
        )
    }

    override fun stop() {
        drivetrain.stop()
        flywheel.stop()
        intake.stopMotor()
        intake.stopServos()
        datalogger.stopLogging()
    }

    private fun deadband(value: Double): Double =
        if (abs(value) < JOYSTICK_DEADBAND) 0.0 else value

    private companion object {
        const val JOYSTICK_DEADBAND = 0.05
    }
}

@TeleOp(name = "FGC: Arcade Drive", group = "FGC Vietnam")
class DrivetrainTeleOp :
    SharedDriveTeleOp(DriveControlMode.ARCADE)

@TeleOp(name = "FGC: Split Arcade Drive", group = "FGC Vietnam")
class DrivetrainSplitArcadeTeleOp :
    SharedDriveTeleOp(DriveControlMode.SPLIT_ARCADE)
