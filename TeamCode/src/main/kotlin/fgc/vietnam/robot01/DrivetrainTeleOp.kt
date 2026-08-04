package fgc.vietnam.robot01

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import fgc.vietnam.robot01.Config.DriverConfig
import fgc.vietnam.robot01.Config.DrivetrainConfig
import fgc.vietnam.robot01.Config.FlywheelConfig.ENABLE_PIDF_TUNING
import fgc.vietnam.robot01.DataLogger.TeleOpDatalogger
import fgc.vietnam.robot01.Hardware.Drivetrain
import fgc.vietnam.robot01.Hardware.Flywheel
import fgc.vietnam.robot01.Hardware.Intake
import fgc.vietnam.robot01.Hardware.IntakeState
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
    private var previousDatalogToggle = false

    private var intakeStop = false

    private lateinit var dashboard: FtcDashboard

    private lateinit var packet: TelemetryPacket

    private var bothButtonsStartTime: Double? = null

    private var bothBumperHold = false


    override fun init() {
        dashboard = FtcDashboard.getInstance()
        drivetrain = Drivetrain(hardwareMap)
        flywheel = Flywheel(hardwareMap)
        intake = Intake(hardwareMap)
        lynxModules = hardwareMap.getAll(LynxModule::class.java)
        telemetry.addData(
            "Status",
            if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
                "A heading hold | B flywheel | LT Intake toggle | LB Outtake hold | RT Hex reverse | D-pad L/R Servos | Start+Y datalog"
            } else {
                "Tank: sticks Y | B flywheel | LT Intake toggle | LB Outtake hold | RT Hex reverse | D-pad L/R Servos"
            },
        )
        val allHubs = hardwareMap.getAll<LynxModule?>(LynxModule::class.java)
        for (module in allHubs) {
            module?.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }
    }

    override fun loop() {
        val loopStartTime = getRuntime()

        packet = TelemetryPacket()

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
                forward = -deadband(combinedLeftStickY).coerceIn(-1.0, 1.0),
                turn = turnInput,
            )

            DriveControlMode.TANK -> drivetrain.driveTank(
                left = -tankLeft,
                right = -tankRight,
            )
        }
        val flywheelState = flywheel.update()

        val flywheelToggle = g1.b || g2.b
        if (flywheelToggle && !previousFlywheelToggle) {
            flywheel.toggle()
        }
        previousFlywheelToggle = flywheelToggle

        // Intake controls:
        // LT = Toggle Intake (ON/OFF)
        // LB = Hold Outtake (momentary override)
        // RT = Hold Hex Motor Reverse (momentary reverse while active)
        val isOuttaking = g1.left_bumper || g2.left_bumper

        val isTransferring = g1.right_bumper || g2.right_bumper

        val bothPressed = isOuttaking && isTransferring

        if (isTransferring && !bothPressed) {
            flywheel.enable()
        } else {
            flywheel.disable()
        }

        if (bothPressed) {
            if (bothButtonsStartTime == null) {
                bothButtonsStartTime = getRuntime()
            }

            if (!bothBumperHold &&
                getRuntime() - bothButtonsStartTime!! >= DriverConfig.intakeStopDely
            ) {
                if (intake.state == IntakeState.OFF) {
                    intake.intake()
                } else {
                    intake.stopMotor()
                }
                bothBumperHold = true
            }
        } else {
            bothButtonsStartTime = null
            bothBumperHold = false
        }

        if (!bothBumperHold) {
            when {
                intake.state == IntakeState.OFF -> intake.stopMotor()
                (isTransferring && flywheel.atTargetVelocity()) -> intake.transfer()
                isOuttaking -> intake.outtake()
                else -> intake.intake()
            }
        }

        val servoBwd = g1.dpad_down || g2.dpad_down
        val servoFwd = g1.dpad_up || g2.dpad_up

        if (servoBwd) {
            intake.moveServosBackward()
        } else if (servoFwd) {
            intake.moveServosForward()
        } else {
            intake.stopServos()
        }

        // Performance metrics (calculate once, use multiple times)
        val loopTimeMs: Double = (getRuntime() - loopStartTime) * 1000.0
        packet.put("Loop Time (ms)", loopTimeMs)

        telemetry.addData("Loop Time", String.format("%.1f ms", loopTimeMs))

        if (ENABLE_PIDF_TUNING){
            packet.put("Target Velocity", flywheel.getTargetVelocity())
            packet.put("Current Velocity", flywheel.getCurrentVelocity())
            packet.put("At Target Velocity", flywheel.atTargetVelocity())
            packet.put("Shooter Motor Power", flywheel.getPower())

            dashboard.sendTelemetryPacket(packet)
        }

        if (drive != null) {
            // Start + Y  →  toggle full-robot datalog (Arcade / Split Arcade only)
            if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
                val datalogToggle = (g1.start && g1.y) || (g2.start && g2.y)
                if (datalogToggle && !previousDatalogToggle) {
                    datalogger.toggleLogging()
                }
                previousDatalogToggle = datalogToggle

                if (datalogger.isLogging) {
                    val hubTemps = lynxModules.map { hub ->
                        try {
                            hub.getTemperature(TempUnit.CELSIUS)
                        } catch (e: Exception) {
                            Double.NaN
                        }
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
        }

        if (flywheelState == null || drive == null || intake == null) return

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
                flywheelState.leftShooterMotor.rpm,
                flywheelState.leftShooterMotor.velocity,
                flywheelState.targetVelocity,
                flywheelState.leftShooterMotor.currentAmps,
            )
            telemetry.addData(
                "Flywheel M3",
                "rpm=%.0f velocity=%.0f/%.0f ticks/s current=%.2f A",
                flywheelState.rightShooterMotor.rpm,
                flywheelState.rightShooterMotor.velocity,
                flywheelState.targetVelocity,
                flywheelState.rightShooterMotor.currentAmps,
            )

            telemetry.addData(
                "Intake",
                "state=%s hex=%s",
                intake.state,
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

