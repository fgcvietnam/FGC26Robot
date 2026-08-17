package FGC.Vietnam

import Climb
import RoadRunner.Drawing.drawRobot
import android.graphics.Color
import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.TelemetryPacket
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.Vector2d
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.hardware.Blinker
import com.qualcomm.robotcore.hardware.Gamepad
import com.qualcomm.robotcore.hardware.VoltageSensor
import com.qualcomm.robotcore.util.ElapsedTime
import FGC.Vietnam.Config.ClimbConfig
import FGC.Vietnam.Config.DrivetrainConfig
import FGC.Vietnam.Config.FlywheelConfig
import FGC.Vietnam.Config.IntakeConfig
import FGC.Vietnam.Config.RoadRunnerConfig
import FGC.Vietnam.DataLogger.TeleOpDatalogger
import FGC.Vietnam.Utils.MotorTester
import fgc.vietnam.robot01.Hardware.Drivetrain
import fgc.vietnam.robot01.Hardware.Flywheel
import fgc.vietnam.robot01.Hardware.Intake
import fgc.vietnam.robot01.Hardware.IntakeSlidePosition
import fgc.vietnam.robot01.Hardware.IntakeState
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit
import java.util.Locale
import java.util.concurrent.TimeUnit
import kotlin.math.abs

enum class Alliance {
    BLUE,
    RED
}

abstract class CompDriveTeleOp protected constructor(var alliance: Alliance) : OpMode() {
    private lateinit var drivetrain: Drivetrain
    private lateinit var flywheel: Flywheel
    private lateinit var intake: Intake
    private lateinit var climb: Climb
    private lateinit var lynxModules: List<LynxModule>
    private val datalogger = TeleOpDatalogger()

    private var previousHeadingToggle = false
    private var previousDatalogToggle = false
    private var lastTempQueryTimeMs = 0L
    private var cachedHubTemps: List<Double> = emptyList()


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

    private val rumblePattern = Gamepad.RumbleEffect.Builder()
        .addStep(1.0, 0.0, 300)
        .addStep(0.0, 0.0, 150)
        .addStep(0.0, 1.0, 300)
        .addStep(0.0, 0.0, 150)
        .build()



    override fun init() {
        dashboard = FtcDashboard.getInstance()
        drivetrain = Drivetrain(hardwareMap)
        flywheel = Flywheel(hardwareMap)
        intake = Intake(hardwareMap)
        climb = Climb(hardwareMap)
        telemetry.addData(
            "Status",
            "A heading hold | B flywheel | LT Intake toggle | LB Outtake hold | RT Hex reverse | D-pad L/R Servos | Start+Y datalog"
        )
        voltageSensor = hardwareMap.voltageSensor.iterator().next()
        allHubs = hardwareMap.getAll(LynxModule::class.java)
        lynxModules = allHubs
        for (module in allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO)
        }
        val startingPose = when (alliance) {
            Alliance.BLUE -> Pose2d(
                Vector2d(
                    RoadRunnerConfig.BLUE_STARTING_POSITION_X_IN,
                    RoadRunnerConfig.BLUE_STARTING_POSITION_Y_IN
                ),
                Math.toRadians(RoadRunnerConfig.BLUE_STARTING_HEADING)
            )
            Alliance.RED -> Pose2d(
                Vector2d(
                    RoadRunnerConfig.RED_STARTING_POSITION_X_IN,
                    RoadRunnerConfig.RED_STARTING_POSITION_Y_IN
                ),
                Math.toRadians(RoadRunnerConfig.RED_STARTING_HEADING)
            )
        }
        drivetrain.setPose(startingPose)

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

        if (warningActive) {
            setWarning(true)
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS)
        }
    }

    override fun start() {
        autoExtending = true
        autoExtendTimer.reset()
        drivetrain.imu.resetYaw()
    }

    override fun loop() {
        val loopStartTime = getRuntime()

        packet = TelemetryPacket(false)


        val headingToggle = gamepad1.shareWasPressed() || gamepad2.shareWasPressed()
        if (headingToggle && !previousHeadingToggle) {
            drivetrain.toggleHeadingHold()
        }
        previousHeadingToggle = headingToggle

        val combinedLeftStickY = gamepad1.left_stick_y.toDouble()
        val combinedRightStickX = gamepad1.right_stick_x.toDouble()

        val forwardInput = deadband(combinedLeftStickY).coerceIn(-1.0, 1.0)
        val turnInput = deadband(combinedRightStickX).coerceIn(-1.0, 1.0)

        val drive = drivetrain.drive(forwardInput, turnInput, false, voltageSensor.voltage)

        val isOuttaking = gamepad1.left_bumper || gamepad2.left_bumper
        val isTransferring = gamepad1.right_bumper || gamepad2.right_bumper



        if (gamepad1.circleWasPressed() || gamepad2.circleWasPressed()) {
            flywheel.toggle()
        }

        if (gamepad1.squareWasPressed() || gamepad2.squareWasPressed()){
            intake.toggle()
        }

        when {
            isTransferring -> intake.transfer(flywheel.atTargetVelocity())
            isOuttaking -> intake.outtake()
            intake.getIntakeState() == IntakeState.OFF -> intake.stopMotor()
            else -> intake.intake()
        }


        if (flywheel.isEnable()) {
            if (!gamepad1.isRumbling) {
                gamepad1.rumble(
                    1.0,
                    1.0,
                    2000
                )
            }
            if (!gamepad2.isRumbling) {
                gamepad2.rumble(
                    1.0,
                    1.0,
                    2000
                )
            }
        } else if (intake.getIntakeState() == IntakeState.OFF) {
            if (!gamepad1.isRumbling) {
                gamepad1.runRumbleEffect(rumblePattern)
            }
            if (!gamepad2.isRumbling) {
                gamepad2.runRumbleEffect(rumblePattern)
            }
        } else {
            if (gamepad1.isRumbling) {
                gamepad1.stopRumble()
            }
            if (gamepad2.isRumbling) {
                gamepad2.stopRumble()
            }
        }

        if (autoExtending && IntakeConfig.ENABLE_AUTO_EXTENDING) {
            val rightPosition = intake.getRightIntakeSlidePosition()
            val leftPosition = intake.getLeftIntakeSlidePosition()

            val bothHome =
                rightPosition == IntakeSlidePosition.HOME &&
                        leftPosition == IntakeSlidePosition.HOME

            val bothExtending =
                rightPosition == IntakeSlidePosition.EXTENDING &&
                        leftPosition == IntakeSlidePosition.EXTENDING

            val bothExtended =
                rightPosition == IntakeSlidePosition.EXTENDED &&
                        leftPosition == IntakeSlidePosition.EXTENDED

            if (bothHome || bothExtending) {
                intake.moveServosForward()
            }

            if (autoExtendTimer.seconds() >= IntakeConfig.AUTO_EXTEND_TIME_SECONDS ||
                bothExtended
            ) {
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

        intake.update()

        val climbInput = gamepad1.right_trigger

        val climbForwardInput = gamepad1.right_trigger
        val climbHoldInput = gamepad1.left_trigger

        if (climbForwardInput > ClimbConfig.TRIGGER_DEADBAND) {
            climb.climbForward(abs(climbForwardInput))
        } else if (climbHoldInput > ClimbConfig.TRIGGER_DEADBAND) {
            climb.updateHoldPower(abs(climbHoldInput))
        } else if (gamepad1.touchpad) {
            climb.climbBackward(1.0)
        } else {
            climb.stop()
        }

        if (gamepad1.triangle || gamepad2.triangle){
            climb.climbExtend()
        } else if ((gamepad1.triangle && gamepad1.touchpad) || (gamepad2.triangle && gamepad2.touchpad)) {
            climb.climbRetract()
        } else {
            climb.climbExtendStop()
        }



        // Performance metrics (calculate once, use multiple times)
        val loopTimeMs: Double = (getRuntime() - loopStartTime) * 1000.0
        packet.put("Loop Time (ms)", loopTimeMs)

        telemetry.addData("Loop Time", String.format(Locale.US,"%.1f ms", loopTimeMs))

        val pose = drivetrain.getPose()

        telemetry.addData("Pose X (mm)", pose.position.x)
        telemetry.addData("Pose Y (mm)", pose.position.y)
        telemetry.addData("Pose Heading (deg)", Math.toDegrees(pose.heading.toDouble()))

        packet.put("Pose X (mm)", pose.position.x)
        packet.put("Pose Y (mm)", pose.position.y)
        packet.put(
            "Pose Heading (deg)",
            Math.toDegrees(pose.heading.toDouble())
        )

        drawField(packet)

        val poseInches = Pose2d(
            Vector2d(
                pose.position.x / 25.4,
                pose.position.y / 25.4
            ),
            pose.heading
        )

        drawRobot(packet.fieldOverlay(), poseInches)

        // Get heading correction data for dashboard

        if (ClimbConfig.CLIMB_DEBUG){
            packet.put("Distance (mm): ", climb.getDistance())
            packet.put("Motor Above Power: ", climb.getMotorAbovePower())
            packet.put("Motor Below Power: ", climb.getMotorBelowPower())
            packet.put("Climb State: ", climb.getClimbState())
            packet.put("Servo power: ", climb.getServoPower())
        }

        if  (DrivetrainConfig.ENABLE_ACTIVE_HEADING_TUNING) {
            val headingData: Drivetrain.HeadingCorrectionData = drivetrain.getHeadingCorrectionData()
            packet.put("Heading Target (deg)", headingData.targetHeadingDeg);
            packet.put("Heading Actual (deg)", headingData.currentHeadingDeg);
            packet.put("Heading Error (deg)", headingData.headingErrorDeg);
            packet.put("Correction Power", headingData.correctionPower);

            // Status indicators
            packet.put("Heading Active", headingData.isActive);
            packet.put("Heading Pending", headingData.isPending);
            packet.put("Tilt Safety", headingData.tiltingSafety);
            packet.put("Custom Heading (deg)", headingData.customHeadingDeg);
        }

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
            packet.put("Both Limits Pressed (Home)", intake.bothLimitsPressed())
            packet.put("Limit Sync State", intake.getLimitSyncState())
            packet.put("Right Magnetic Switch is Pressed", intake.rightMagnetRegistered())
            packet.put("Left Magnetic Switch is Pressed", intake.leftMagnetRegistered())
            packet.put("Both Magnets Detected (Extended)", intake.bothMagnetsRegistered())
            packet.put("Magnet Sync State", intake.getMagnetSyncState())
            packet.put("Right Magnetic Switch is Confirmed", intake.rightMagnetConfirmed())
            packet.put("Left Magnetic Switch is Confirmed", intake.leftMagnetConfirmed())
            packet.put("Both Magnets Confirmed", intake.bothMagnetsConfirmed())
            packet.put("Right Intake Slide State", intake.getRightIntakeSlidePosition().name)
            packet.put("Left Intake Slide State", intake.getLeftIntakeSlidePosition().name)
            packet.put("Slides Synchronized", intake.areSlidesSynchronized())
            packet.put("Slide Skew State", intake.getSlideSkewState())
            packet.put("Servo Right Below Power", intake.servoRightBelowPower)
            packet.put("Servo Right Above Power", intake.servoRightAbovePower)
            packet.put("Servo Left Below Power", intake.servoLeftBelowPower)
            packet.put("Servo Left Above Power", intake.servoLeftAbovePower)
        }

        if (drive != null) {
            packet.put("Heading Deg", drive.heading)
            packet.put("Target Heading Deg", drive.targetHeading)
            packet.put("Heading Error Deg", drive.headingError)
            packet.put("Left Ticks", drive.leftEncoderPosition)
            packet.put("Right Ticks", drive.rightEncoderPosition)
            packet.put("Left Vel (ticks/s)", drive.leftActualVelocity)
            packet.put("Right Vel (ticks/s)", drive.rightActualVelocity)
            packet.put("Left Motor Power", drive.leftMotorPower)
            packet.put("Right Motor Power", drive.rightMotorPower)
            packet.put("Battery (V)", voltageSensor.voltage)
        }

        dashboard.sendTelemetryPacket(packet)

        if (drive != null) {
            val datalogToggle = (gamepad1.start && gamepad1.y) || (gamepad2.start && gamepad2.y)
            if (datalogToggle && !previousDatalogToggle) {
                datalogger.toggleLogging()
            }
            previousDatalogToggle = datalogToggle

            if (datalogger.isLogging) {
                val nowMs = System.currentTimeMillis()
                if (nowMs - lastTempQueryTimeMs >= 2000L || cachedHubTemps.isEmpty()) {
                    lastTempQueryTimeMs = nowMs
                    cachedHubTemps = lynxModules.map { hub ->
                        try {
                            hub.getTemperature(TempUnit.CELSIUS)
                        } catch (e: Exception) {
                            Double.NaN
                        }
                    }
                }
                datalogger.writeRow(
                    gamepad1 = gamepad1,
                    gamepad2 = gamepad2,
                    drive = drive,
                    flywheel = flywheelState,
                    intake = intake,
                    hubTemperaturesCelsius = cachedHubTemps,
                )
            }
        }

        if (drive == null || intake == null) return

        telemetry.addData(
            "Input",
            "forward=%.2f→%.2f turn=%.2f hold=%s",
            drive.requestedForward,
            drive.limitedForward,
            drive.requestedTurn,
            if (drive.headingHoldEnabled) "ON" else "OFF",
        )

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

        if (flywheelState != null && flywheelState.enabled) {
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
                "Flywheel wheel",
                "rim=%.1f m/s",
                flywheelState.surfaceSpeedMetersPerSecond,
            )
        } else {
            telemetry.addData("Flywheel", "OFF")
        }

        telemetry.addData(
            "Intake",
            "state=%s hex=%.2f pwr=%.2f",
            intake.getIntakeState().name,
            intake.hexMotorPower,
            intake.motorPower,
        )

        telemetry.addData(
            "Linear Slide Sync",
            "Pos: %s | Mag: %s | Home: %s",
            intake.getSlideSkewState(),
            intake.getMagnetSyncState(),
            intake.getLimitSyncState(),
        )

        if (IntakeConfig.INTAKE_DEBUG) {
            telemetry.addData(
                "Slide Sensors",
                "Mag [L:%s R:%s Both:%s] | Lim [L:%s R:%s Both:%s]",
                if (intake.leftMagnetRegistered()) "HIT" else "—",
                if (intake.rightMagnetRegistered()) "HIT" else "—",
                if (intake.bothMagnetsRegistered()) "YES" else "NO",
                if (intake.leftLimitSwitchIsPressed()) "HIT" else "—",
                if (intake.rightLimitSwitchIsPressed()) "HIT" else "—",
                if (intake.bothLimitsPressed()) "YES" else "NO",
            )
        }

        telemetry.addData(
            "Datalog",
            "%s rows=%d",
            if (datalogger.isLogging) "● REC" else "○ off",
            datalogger.rowCount,
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

    fun drawField(packet: TelemetryPacket) {
        val canvas = packet.fieldOverlay()
        canvas.drawImage(
            "/images/fgc_field.png",
            -RoadRunnerConfig.FIELD_SIZE_IN / 2,
            -RoadRunnerConfig.FIELD_SIZE_IN / 2,
            RoadRunnerConfig.FIELD_SIZE_IN,
            RoadRunnerConfig.FIELD_SIZE_IN
        )
    }

}
