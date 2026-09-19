package FGC.Vietnam

import FGC.Vietnam.Hardware.Climb
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
import FGC.Vietnam.Utils.MotorTester
import FGC.Vietnam.Hardware.Drivetrain
import FGC.Vietnam.Hardware.Flywheel
import FGC.Vietnam.Hardware.Intake
import FGC.Vietnam.Hardware.IntakeSlidePosition
import FGC.Vietnam.Hardware.IntakeState
import java.util.Locale
import java.util.concurrent.TimeUnit
import kotlin.math.abs

enum class Alliance {
    BLUE,
    RED
}

abstract class CompDriveTeleOp protected constructor(
    var alliance: Alliance,
    var transferDatalogOnly: Boolean = false
) : OpMode() {
    private lateinit var drivetrain: Drivetrain
    private lateinit var flywheel: Flywheel
    private lateinit var intake: Intake
    private lateinit var climb: Climb
    private lateinit var lynxModules: List<LynxModule>

    private var previousHeadingToggle = false


    private lateinit var dashboard: FtcDashboard

    private lateinit var packet: TelemetryPacket


    private lateinit var voltageSensor: VoltageSensor

    private val homingTimer = ElapsedTime()

    private var motorTestTimer = ElapsedTime()
    private var autoExtendTimer = ElapsedTime()
    private var warningActive = false
    private var autoExtending = true

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
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL)
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
        allHubs.forEach { it.clearBulkCache() }
        val loopStartTime = getRuntime()

        packet = TelemetryPacket(false)


        val headingToggle = gamepad1.shareWasPressed() || gamepad2.shareWasPressed()
        if (headingToggle && !previousHeadingToggle) {
            drivetrain.toggleHeadingHold()
        }
        previousHeadingToggle = headingToggle

        val forwardInput = deadband(gamepad1.left_stick_y.toDouble()).coerceIn(-1.0, 1.0)
        val turnInput = deadband(-gamepad1.right_stick_x.toDouble()).coerceIn(-1.0, 1.0)

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

        flywheel.update(voltageSensor.voltage)

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

        if (gamepad1.touchpadWasPressed()){
            climb.climbHookOpen()
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
            packet.put("Servo Left Below Power", intake.servoLeftBelowPower)
        }


        dashboard.sendTelemetryPacket(packet)
    }

    override fun stop() {
        drivetrain.stop()
        flywheel.stop()
        intake.stopMotor()
        intake.stopServos()
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
