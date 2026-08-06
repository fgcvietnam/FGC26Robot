package fgc.vietnam.robot01

import MotorTester
import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Blinker
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import fgc.vietnam.robot01.Config.ClimbConfig
import fgc.vietnam.robot01.Config.DrivetrainConfig
import fgc.vietnam.robot01.Config.FlywheelConfig
import fgc.vietnam.robot01.Config.IntakeConfig
import fgc.vietnam.robot01.DataLogger.TeleOpDatalogger
import fgc.vietnam.robot01.Hardware.Climb
import fgc.vietnam.robot01.Hardware.Drivetrain
import fgc.vietnam.robot01.Hardware.Flywheel
import fgc.vietnam.robot01.Hardware.Intake
import fgc.vietnam.robot01.Hardware.IntakeSlidePosition
import fgc.vietnam.robot01.Hardware.ClimbState
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.Locale
import java.util.concurrent.TimeUnit
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
    private lateinit var climb: Climb
    private lateinit var lynxModules: List<LynxModule>
    private val datalogger = TeleOpDatalogger()

    private var previousHeadingToggle = false
    private var previousDatalogToggle = false


    private lateinit var dashboard: FtcDashboard

    private lateinit var packet: TelemetryPacket


    private lateinit var voltageSensor: VoltageSensor

    private val homingTimer = ElapsedTime()

    private var motorTestTimer = ElapsedTime()
    private var autoExtendTimer = ElapsedTime()
    private var warningActive = false
    private var autoExtending = true

    lateinit var intakeTester: MotorTester
    lateinit var leftFlywheelTester: MotorTester
    lateinit var rightFlywheelTester: MotorTester

    lateinit var allHubs: List<LynxModule>

    var testState = 0

    private val okPattern = listOf(

        Blinker.Step(Color.GREEN, 1, TimeUnit.SECONDS)

    )

    private val warningPattern = listOf(

        Blinker.Step(Color.RED, 300, TimeUnit.MILLISECONDS),

        Blinker.Step(Color.BLACK, 300, TimeUnit.MILLISECONDS),

    )

    private val errorPattern = listOf(

        Blinker.Step(Color.RED, 150, TimeUnit.MILLISECONDS),

        Blinker.Step(Color.BLACK, 150, TimeUnit.MILLISECONDS),

        Blinker.Step(Color.RED, 150, TimeUnit.MILLISECONDS),

        Blinker.Step(Color.BLACK, 700, TimeUnit.MILLISECONDS)

    )



    override fun init() {
        dashboard = FtcDashboard.getInstance()
        drivetrain = Drivetrain(hardwareMap)
        flywheel = Flywheel(hardwareMap)
        intake = Intake(hardwareMap)
        climb = Climb(hardwareMap)
        telemetry.addData(
            "Status",
            if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
                "A heading hold | B flywheel | LT Intake toggle | LB Outtake hold | RT Hex reverse | D-pad L/R Servos | Start+Y datalog"
            } else {
                "Tank: sticks Y | B flywheel | LT Intake toggle | LB Outtake hold | RT Hex reverse | D-pad L/R Servos"
            },
        )
        voltageSensor = hardwareMap.voltageSensor.iterator().next()
        allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (module in allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }
        homingTimer.reset()
        autoExtendTimer.reset()

        intakeTester = MotorTester(intake.getHexMotor()).apply {
            expectedDirection = intake.getHexMotor().direction
        }
        leftFlywheelTester = MotorTester(flywheel.getLeftShooterMotor()).apply {
            expectedDirection = flywheel.getLeftShooterMotor().direction
        }
        rightFlywheelTester = MotorTester(flywheel.getRightShooterMotor()).apply {
            expectedDirection = flywheel.getRightShooterMotor().direction
        }
    }

    override fun init_loop() {
        if (IntakeConfig.EXTENSION_HOMING){
            if (intake.getRightIntakeSlidePosition() != IntakeSlidePosition.HOME ||
                intake.getLeftIntakeSlidePosition() != IntakeSlidePosition.HOME
            ) {
                if (homingTimer.seconds() > IntakeConfig.INTAKE_HOMING_TIMEOUT_SECONDS) {
                    intake.stopServos()
                    telemetry.addLine("⚠️❌ Intake Homing Timed Out ❌⚠️")
                    warningActive = true
                } else {
                    intake.moveServosBackward()
                }
            } else {
                intake.stopServos()
                telemetry.addLine("✅ Homing complete ✅")
            }
        } else {
            intake.stopServos()
        }
//
//        when(testState) {
//
//            0 -> {
//                intakeTester.start()
//                testState++
//            }
//
//            1 -> {
//                intakeTester.update()
//                if (intakeTester.finished()) {
//                    val result = intakeTester.result!!
//                    telemetry.addData("Intake", result)
//                    if (!result.passed) {
//                        warningRumbling = true
//                    }
//                    testState++
//                }
//            }
//
//            2 -> {
//                leftFlywheelTester.start()
//                testState++
//            }
//
//            3 -> {
//                leftFlywheelTester.update()
//
//                if (leftFlywheelTester.finished()) {
//                    val result = intakeTester.result!!
//                    telemetry.addData("Flywheel L", leftFlywheelTester.result)
//                    testState++
//                }
//            }
//
//            4 -> {
//                rightFlywheelTester.start()
//                testState++
//            }
//
//            5 -> {
//                rightFlywheelTester.update()
//
//                if (rightFlywheelTester.finished()) {
//                    telemetry.addData("Flywheel R", rightFlywheelTester.result)
//                }
//                testState = -1
//            }
//        }



        if (warningActive) {
            setWarning(true)
            if (!gamepad1.isRumbling) {
                gamepad1.rumble(1.0, 1.0, 1000)
            }
            if (!gamepad2.isRumbling) {
                gamepad2.rumble(1.0, 1.0, 1000)
            }
        }
    }

    override fun start() {
        autoExtending = true
        autoExtendTimer.reset()
    }

    override fun loop() {
        val loopStartTime = getRuntime()

        packet = TelemetryPacket()


        val headingToggle = gamepad1.a || gamepad2.a
        if (
            (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) &&
            headingToggle &&
            !previousHeadingToggle
        ) {
            drivetrain.toggleHeadingHold()
        }
        previousHeadingToggle = headingToggle

        val combinedRightStickY = -gamepad1.right_stick_y.toDouble()
        val combinedRightStickX = -gamepad1.right_stick_x.toDouble()
        val combinedLeftStickY = -gamepad1.left_stick_y.toDouble()

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

        val isOuttaking = gamepad1.left_bumper || gamepad2.left_bumper
        val isTransferring = gamepad1.right_bumper || gamepad2.right_bumper


        if (gamepad1.squareWasPressed() || gamepad2.squareWasPressed()){
            intake.toggle()
        }


        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            flywheel.toggle()
        }

        when {
            intake.getIntakeState() == ClimbState.OFF -> intake.stopMotor()
            isTransferring -> intake.transfer(flywheel.atTargetVelocity())
            isOuttaking -> intake.outtake()
            else -> intake.intake()
        }
        if (flywheel.isEnable()){
            if (!gamepad1.isRumbling) {
                gamepad1.rumble(1000)
            }
        }

        if (autoExtending && IntakeConfig.ENABLE_AUTO_EXTENDING) {
            intake.moveServosForward()

            if (autoExtendTimer.seconds() >= IntakeConfig.AUTO_EXTEND_TIME_SECONDS) {
                intake.stopServos()
                autoExtending = false
            }
        } else {
            val servoBwd = gamepad1.dpad_down || gamepad2.dpad_down
            val servoFwd = gamepad1.dpad_up || gamepad2.dpad_up

            when {
                servoBwd -> intake.moveServosBackward()
                servoFwd -> intake.moveServosForward()
                else -> intake.stopServos()
            }
        }

        val flywheelState = flywheel.update(voltageSensor.voltage)

        val climbInput = gamepad1.right_trigger - gamepad1.left_trigger

        val power = abs(climbInput / ClimbConfig.CUT_OFF_INPUT).coerceAtMost(1.0)

        if (climbInput > ClimbConfig.TRIGGER_DEADBAND){
            climb.climbForward(power)
        } else if (climbInput < ClimbConfig.TRIGGER_DEADBAND){
            climb.climbBackward(power)
        } else {
            climb.stop()
        }

        if (gamepad1.triangleWasPressed() || gamepad2.triangleWasPressed()){
            climb.climbExtend()
        } else {
            climb.climbExtendStop()
        }



        // Performance metrics (calculate once, use multiple times)
        val loopTimeMs: Double = (getRuntime() - loopStartTime) * 1000.0
        packet.put("Loop Time (ms)", loopTimeMs)

        telemetry.addData("Loop Time", String.format(Locale.US,"%.1f ms", loopTimeMs))

        if (FlywheelConfig.ENABLE_PIDF_TUNING){
            packet.put("Target Velocity", flywheel.getTargetVelocity())
            packet.put("Current Velocity", flywheel.getCurrentVelocity())
            packet.put("At Target Velocity", flywheel.atTargetVelocity())
            packet.put("Shooter Motor Power / Output", flywheel.getPower())
            packet.put("last error", flywheel.shooterPIDF.getLastError())
            packet.put("last P output", flywheel.shooterPIDF.getLastPOutput())
            packet.put("last PID output", flywheel.shooterPIDF.getLastPIDOutput())
            packet.put("last FF output", flywheel.shooterPIDF.getFFoutput())
        }
        if (IntakeConfig.ENABLE_LIMIT_SWITCH_AND_MAGNETIC_TESTING){
            packet.put("Right Limit Switch is Pressed", intake.rightLimitSwitchIsPressed())
            packet.put("Left Limit Switch is Pressed", intake.leftLimitSwitchIsPressed())
            packet.put("Right Intake Slide State ", intake.getRightIntakeSlidePosition().name)
            packet.put("Left Intake Slide State ", intake.getLeftIntakeSlidePosition().name)
            packet.put("Servo Right Below Power", intake.servoRightBelowPower)
            packet.put("Servo Right Above Power", intake.servoRightAbovePower)
            packet.put("Servo Left Below Power", intake.servoLeftBelowPower)
            packet.put("Servo Left Above Power", intake.servoLeftAbovePower)
        }

        dashboard.sendTelemetryPacket(packet)

        if (drive != null) {
            // Start + Y  →  toggle full-robot datalog (Arcade / Split Arcade only)
            if (controlMode == DriveControlMode.ARCADE || controlMode == DriveControlMode.SPLIT_ARCADE) {
                val datalogToggle = (gamepad1.start && gamepad1.y) || (gamepad2.start && gamepad2.y)
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
                intake.getIntakeState(),
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
        if (abs(value) < DrivetrainConfig.JOYSTICK_DEADBAND) 0.0 else value

    fun setWarning(active: Boolean) {
        if (active == warningActive) return
        warningActive = active
        val pattern = if (active) warningPattern else okPattern
        allHubs.forEach { it.setPattern(pattern) }
    }
}

