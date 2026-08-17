package fgc.vietnam.robot01.Hardware

import FGC.Vietnam.Config.DrivetrainConfig
import FGC.Vietnam.Config.RobotConfig
import FGC.Vietnam.Hardware.Vision
import FGC.Vietnam.Utils.FeedForward
import FGC.Vietnam.Utils.PIDFController
import RoadRunner.Localizer
import RoadRunner.PoseEstimator
import com.acmerobotics.roadrunner.DualNum
import com.acmerobotics.roadrunner.Pose2d
import com.acmerobotics.roadrunner.PoseVelocity2d
import com.acmerobotics.roadrunner.Rotation2d
import com.acmerobotics.roadrunner.TankKinematics
import com.acmerobotics.roadrunner.Time
import com.acmerobotics.roadrunner.Vector2d
import com.acmerobotics.roadrunner.ftc.OverflowEncoder
import com.acmerobotics.roadrunner.ftc.RawEncoder
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.IMU
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min
import kotlin.math.pow
import kotlin.math.sign

internal data class DriveTelemetry(
    val requestedForward: Double,
    val limitedForward: Double,
    val requestedTurn: Double,
    val heading: Double,
    val targetHeading: Double,
    val headingError: Double,
    val headingCorrection: Double,
    val proportionalCorrection: Double,
    val integralCorrection: Double,
    val derivativeCorrection: Double,
    val yawRate: Double,
    val leftTargetVelocity: Double,
    val leftActualVelocity: Double,
    val rightTargetVelocity: Double,
    val rightActualVelocity: Double,
    val leftCurrentAmps: Double,
    val rightCurrentAmps: Double,
    val linearSpeedMmPerSecond: Double,
    val batteryVoltage: Double,
    val headingHoldEnabled: Boolean,
    val leftEncoderPosition: Int,
    val rightEncoderPosition: Int,
    val leftMotorPower: Double,
    val rightMotorPower: Double,
    val pitchDegrees: Double,
    val rollDegrees: Double,
    val pitchRate: Double,
    val rollRate: Double,
    val leftWheelSpeedMmPerSecond: Double,
    val rightWheelSpeedMmPerSecond: Double,
    val tiltingSafety: Boolean,
    val highSpeedTurnBoostActive: Boolean,
    val poseX: Double,
    val poseY: Double,
    val poseHeading: Double,
    val visionActive: Boolean,
    val bestDetectionId: Int
)

internal class Drivetrain(private val hardwareMap: HardwareMap) {
    private val leftMotor = motor(DrivetrainConfig.LEFT_MOTOR_NAME, DcMotorSimple.Direction.REVERSE)
    private val rightMotor = motor(DrivetrainConfig.RIGHT_MOTOR_NAME, DcMotorSimple.Direction.FORWARD)

    val imu = hardwareMap.get(IMU::class.java, DrivetrainConfig.IMU_NAME)


    private var customHeadingRad: Double = 0.0
    private var activeHeadingHoldEnabled: Boolean = true
    private var wasUserTurning = false
    private var lastTurnReleasedTime = 0.0
    private var pendingHeadingLock = false
    private var headingCorrectionStartTime = -1.0

    private val headingController: PIDFController
    
    private var lastLeftPower = 0.0
    private var lastRightPower = 0.0

    private var lastCorrectionPower = 0.0
    private var lastHeadingErrorDeg = 0.0
    private var lastTargetHeadingDeg = 0.0
    private var lastCurrentHeadingDeg = 0.0

    private var lastTiltingSafety = false

    private var highSpeedTurnBoostActive = false

    // RoadRunner Localization
    val localizer: Localizer
    private val kinematics = TankKinematics(DrivetrainConfig.TRACK_WIDTH_MM)

    // Vision and Pose Estimation
    private val vision = Vision(hardwareMap)
    private val poseEstimator = PoseEstimator(Pose2d(Vector2d(0.0, 0.0), Rotation2d.exp(0.0)))
    init {
        val headingFeedforward = FeedForward(
            kS = DrivetrainConfig.ACTIVE_HEADING_KS,
        )
        headingController = PIDFController(
            kP = DrivetrainConfig.ACTIVE_HEADING_KP,
            kI = DrivetrainConfig.ACTIVE_HEADING_KI,
            kD = DrivetrainConfig.ACTIVE_HEADING_KD,
            feedForward = headingFeedforward
        )
        headingController.setOutputLimits(-1.0, 1.0)
        headingController.setIntegralLimits(-0.5, 0.5)
        headingController.setIntegralWindupProtection(true)

        imu.initialize(IMU.Parameters(RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        )))

        imu.resetYaw()

//         Initialize localizer using drive encoders
        localizer = object : Localizer {
            private val leftEnc = OverflowEncoder(RawEncoder(leftMotor))
            private val rightEnc = OverflowEncoder(RawEncoder(rightMotor))
            private var pose = Pose2d(Vector2d(0.0, 0.0), Rotation2d.exp(0.0))
            private var lastLeftPos = 0.0
            private var lastRightPos = 0.0
            private var initialized = false

            override fun setPose(pose: Pose2d) {
                this.pose = pose

                val leftRead = leftEnc.getPositionAndVelocity()
                val rightRead = rightEnc.getPositionAndVelocity()

                lastLeftPos = leftRead.position.toDouble()
                lastRightPos = rightRead.position.toDouble()

                initialized = true
            }

            override fun getPose(): Pose2d = pose

            override fun update(): PoseVelocity2d {
                val leftRead = leftEnc.getPositionAndVelocity()
                val rightRead = rightEnc.getPositionAndVelocity()

                val currentLeftPos = leftRead.position.toDouble()
                val currentRightPos = rightRead.position.toDouble()
                val currentLeftVel = (leftRead.velocity ?: 0).toDouble()
                val currentRightVel = (rightRead.velocity ?: 0).toDouble()

                if (!initialized) {
                    lastLeftPos = currentLeftPos
                    lastRightPos = currentRightPos
                    initialized = true
                    return PoseVelocity2d(Vector2d(0.0, 0.0), 0.0)
                }

                val leftPosDelta = currentLeftPos - lastLeftPos
                val rightPosDelta = currentRightPos - lastRightPos

                val twist = kinematics.forward(TankKinematics.WheelIncrements(
                    DualNum<Time>(doubleArrayOf(leftPosDelta, currentLeftVel)).times(DrivetrainConfig.millimetersPerEncoderTick),
                    DualNum<Time>(doubleArrayOf(rightPosDelta, currentRightVel)).times(DrivetrainConfig.millimetersPerEncoderTick)
                ))

                lastLeftPos = currentLeftPos
                lastRightPos = currentRightPos

                pose = pose.plus(twist.value())
                return twist.velocity().value()
            }
        }
    }

    private fun motor(name: String, direction: DcMotorSimple.Direction): DcMotorEx =
        hardwareMap.get(DcMotorEx::class.java, name).apply {
            this.direction = direction
            zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
            mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
            mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        }

    private fun updateGlobalPose(currentHeadingRad: Double) {
        if (DrivetrainConfig.LOCALIZER_ENABLE) {
            localizer.update()

//         Use IMU heading for stability in the pose estimator
            val imuHeading = Rotation2d.exp(currentHeadingRad)

            poseEstimator.updateOdometry(localizer.getPose(), imuHeading)

            // Correct drift using vision if an AprilTag is detected
            val detection = vision.getBestDetection()
            if (detection != null) {
                val visionPose = vision.getRobotPose(detection)

                if (visionPose != null) {
                    poseEstimator.addVisionMeasurement(
                        visionPose,
                        detection
                    )
                }
            }
        }
    }

    private fun startHeadingCorrectionTimer() {
        headingCorrectionStartTime = if (DrivetrainConfig.DPAD_ORIENTATION_TIMEOUT_SECONDS > 0) {
            System.currentTimeMillis() / 1000.0
        } else {
            -1.0
        }
    }

    private fun isHeadingCorrectionTimedOut(currentTime: Double): Boolean {
        if (DrivetrainConfig.DPAD_ORIENTATION_TIMEOUT_SECONDS <= 0 || headingCorrectionStartTime <= 0) return false
        return (currentTime - headingCorrectionStartTime) > DrivetrainConfig.DPAD_ORIENTATION_TIMEOUT_SECONDS
    }

    private fun shapeInput(input: Double, deadband: Double, expo: Double): Double {
        val absInput = abs(input)
        if (absInput <= deadband) return 0.0
        val sign = sign(input)
        val scaled = (absInput - deadband) / (1.0 - deadband)
        return sign * ((1.0 - expo) * scaled + expo * scaled.pow(3))
    }

    fun drive(forward: Double, turn: Double, precisionMode: Boolean = false, batteryVoltage: Double): DriveTelemetry? {
        val robotAngles = imu.robotYawPitchRollAngles
        val currentHeadingRad = robotAngles.getYaw(AngleUnit.RADIANS)

        updateGlobalPose(currentHeadingRad)
        val currentPose = poseEstimator.pose
        val currentTime = System.currentTimeMillis() / 1000.0

        val forwardShaped = shapeInput(forward, DrivetrainConfig.FORWARD_DEADBAND, DrivetrainConfig.FORWARD_EXPO)
        val turnShaped = shapeInput(turn, DrivetrainConfig.TURN_DEADBAND, DrivetrainConfig.TURN_EXPO)

        var speedMultiplier = DrivetrainConfig.DRIVE_SPEED_MULTIPLIER
        var turnMultiplier = DrivetrainConfig.TURN_SPEED_MULTIPLIER
        if (precisionMode) {
            speedMultiplier *= DrivetrainConfig.PRECISION_MODE_MULTIPLIER
            turnMultiplier *= DrivetrainConfig.PRECISION_MODE_MULTIPLIER
        }

        val forwardAbs = abs(forwardShaped)
        if (forwardAbs > DrivetrainConfig.TURN_BOOST_START) {
            val normForward = min(1.0, (forwardAbs - DrivetrainConfig.TURN_BOOST_START) / (DrivetrainConfig.TURN_BOOST_END - DrivetrainConfig.TURN_BOOST_START))
            turnMultiplier *= (1.0 + (DrivetrainConfig.HIGH_SPEED_TURN_BOOST - 1.0) * normForward.pow(DrivetrainConfig.TURN_BOOST_EXPONENT))
            highSpeedTurnBoostActive = normForward > 0.1
        } else {
            highSpeedTurnBoostActive = false
        }

        val driveForward = forwardShaped * speedMultiplier
        val userTurnInput = turnShaped * turnMultiplier
        var finalTurn = userTurnInput

        val robotTilting =
            abs(robotAngles.getPitch(AngleUnit.DEGREES)) >
                    DrivetrainConfig.MAX_TILT_FOR_HEADING_CORRECTION_DEG ||
                    abs(robotAngles.getRoll(AngleUnit.DEGREES)) >
                    DrivetrainConfig.MAX_TILT_FOR_HEADING_CORRECTION_DEG
        // Update heading telemetry state every drive cycle
        lastCurrentHeadingDeg = Math.toDegrees(currentHeadingRad)
        lastTargetHeadingDeg = Math.toDegrees(customHeadingRad)
        lastTiltingSafety = robotTilting

        val userTurning = abs(userTurnInput) > DrivetrainConfig.ACTIVE_HEADING_TURN_DEADBAND

        if (DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION && !robotTilting) {
            if (!userTurning && wasUserTurning) {
                lastTurnReleasedTime = currentTime
                pendingHeadingLock = true
            } else if (userTurning) {
                activeHeadingHoldEnabled = false
                pendingHeadingLock = false
            }

            if (pendingHeadingLock && !userTurning && (currentTime - lastTurnReleasedTime >= DrivetrainConfig.ACTIVE_HEADING_SETTLE_TIME_SECONDS)) {
                if (!activeHeadingHoldEnabled) {
                    customHeadingRad = currentHeadingRad
                    activeHeadingHoldEnabled = true
                    headingController.reset()
                }
                pendingHeadingLock = false
            }
        } else {
            activeHeadingHoldEnabled = false
            pendingHeadingLock = false
        }
        wasUserTurning = userTurning

        val isTranslating = abs(driveForward) > DrivetrainConfig.FORWARD_DEADBAND
        if (activeHeadingHoldEnabled && isTranslating && !robotTilting && !isHeadingCorrectionTimedOut(currentTime)) {
            val error = minimalAngleDifference(customHeadingRad, currentHeadingRad)

            lastHeadingErrorDeg = Math.toDegrees(error)
            lastTargetHeadingDeg = Math.toDegrees(customHeadingRad)
            lastCurrentHeadingDeg = Math.toDegrees(currentHeadingRad)

            if (abs(Math.toDegrees(error)) > DrivetrainConfig.ACTIVE_HEADING_HOLD_DEADBAND_DEG) {
                lastCorrectionPower = headingController.calculate(error, 0.0)
                finalTurn = lastCorrectionPower
            } else {
                lastCorrectionPower = 0.0
                headingController.reset()
            }
        } else {
            lastCorrectionPower = 0.0
            headingController.reset()
        }

        var left = driveForward - finalTurn
        var right = driveForward + finalTurn
        val maxPower = max(1.0, max(abs(left), abs(right)))
        left /= maxPower
        right /= maxPower

        setMotorPowersSmart(left, right, batteryVoltage)

        if (DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION){
            headingController.setPIDF(
                kP = DrivetrainConfig.ACTIVE_HEADING_KP,
                kI = DrivetrainConfig.ACTIVE_HEADING_KI,
                kD = DrivetrainConfig.ACTIVE_HEADING_KD,
                FeedForward(
                    kS = DrivetrainConfig.ACTIVE_HEADING_KS
                )
            )
        }

        if (RobotConfig.DATALOG_ENABLED || DrivetrainConfig.DATALOG_ENABLED) {
            val angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
            val bestDetection = vision.getBestDetection()
            return DriveTelemetry(
                requestedForward = forward,
                limitedForward = driveForward,
                requestedTurn = turn,
                heading = Math.toDegrees(currentHeadingRad),
                targetHeading = Math.toDegrees(customHeadingRad),
                headingError = Math.toDegrees(
                    minimalAngleDifference(customHeadingRad, currentHeadingRad)
                ),
                headingCorrection = lastCorrectionPower,
                proportionalCorrection = headingController.lastProportional,
                integralCorrection = headingController.lastIntegral,
                derivativeCorrection = headingController.lastDerivative,
                yawRate = angularVelocity.zRotationRate.toDouble(),
                leftTargetVelocity = left * leftMotor.motorType.achieveableMaxTicksPerSecond,
                leftActualVelocity = leftMotor.velocity,
                rightTargetVelocity = right * rightMotor.motorType.achieveableMaxTicksPerSecond,
                rightActualVelocity = rightMotor.velocity,
                leftCurrentAmps = leftMotor.getCurrent(CurrentUnit.AMPS),
                rightCurrentAmps = rightMotor.getCurrent(CurrentUnit.AMPS),
                linearSpeedMmPerSecond = (leftMotor.velocity + rightMotor.velocity) / 2.0 * DrivetrainConfig.millimetersPerEncoderTick,
                batteryVoltage = batteryVoltage,
                headingHoldEnabled = activeHeadingHoldEnabled,
                leftEncoderPosition = leftMotor.currentPosition,
                rightEncoderPosition = rightMotor.currentPosition,
                leftMotorPower = left,
                rightMotorPower = right,
                pitchDegrees = robotAngles.getPitch(AngleUnit.DEGREES),
                rollDegrees = robotAngles.getRoll(AngleUnit.DEGREES),
                pitchRate = angularVelocity.xRotationRate.toDouble(),
                rollRate = angularVelocity.yRotationRate.toDouble(),
                leftWheelSpeedMmPerSecond = leftMotor.velocity * DrivetrainConfig.millimetersPerEncoderTick,
                rightWheelSpeedMmPerSecond = rightMotor.velocity * DrivetrainConfig.millimetersPerEncoderTick,
                tiltingSafety = robotTilting,
                highSpeedTurnBoostActive = highSpeedTurnBoostActive,
                poseX = currentPose.position.x,
                poseY = currentPose.position.y,
                poseHeading = Math.toDegrees(currentPose.heading.toDouble()),
                visionActive = bestDetection != null,
                bestDetectionId = bestDetection?.id ?: -1
            )
        }
        return null
    }

    /**
     * Direct drive method for autonomous / automated test routines.
     * Bypasses joystick deadband and expo shaping, with capped heading correction.
     */
    fun driveDirect(
        forward: Double,
        turn: Double = 0.0,
        batteryVoltage: Double,
        holdHeading: Boolean = true
    ): DriveTelemetry? {
        val robotAngles = imu.robotYawPitchRollAngles
        val currentHeadingRad = robotAngles.getYaw(AngleUnit.RADIANS)

        updateGlobalPose(currentHeadingRad)
        val currentPose = poseEstimator.pose

        val driveForward = forward.coerceIn(-1.0, 1.0)
        var finalTurn = turn.coerceIn(-1.0, 1.0)

        val robotTilting =
            abs(robotAngles.getPitch(AngleUnit.DEGREES)) > DrivetrainConfig.MAX_TILT_FOR_HEADING_CORRECTION_DEG ||
            abs(robotAngles.getRoll(AngleUnit.DEGREES)) > DrivetrainConfig.MAX_TILT_FOR_HEADING_CORRECTION_DEG

        lastCurrentHeadingDeg = Math.toDegrees(currentHeadingRad)
        lastTargetHeadingDeg = Math.toDegrees(customHeadingRad)
        lastTiltingSafety = robotTilting

        if (holdHeading && DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION && !robotTilting && abs(driveForward) > 0.01) {
            val error = minimalAngleDifference(customHeadingRad, currentHeadingRad)
            lastHeadingErrorDeg = Math.toDegrees(error)

            if (abs(Math.toDegrees(error)) > DrivetrainConfig.ACTIVE_HEADING_HOLD_DEADBAND_DEG) {
                val correction = headingController.calculate(error, 0.0).coerceIn(-0.35, 0.35)
                lastCorrectionPower = correction
                finalTurn += correction
            } else {
                lastCorrectionPower = 0.0
                headingController.reset()
            }
        } else {
            lastCorrectionPower = 0.0
            headingController.reset()
        }

        var left = driveForward - finalTurn
        var right = driveForward + finalTurn
        val maxPower = max(1.0, max(abs(left), abs(right)))
        left /= maxPower
        right /= maxPower

        setMotorPowersSmart(left, right, batteryVoltage)

        if (DrivetrainConfig.ENABLE_ACTIVE_HEADING_CORRECTION) {
            headingController.setPIDF(
                kP = DrivetrainConfig.ACTIVE_HEADING_KP,
                kI = DrivetrainConfig.ACTIVE_HEADING_KI,
                kD = DrivetrainConfig.ACTIVE_HEADING_KD,
                FeedForward(
                    kS = DrivetrainConfig.ACTIVE_HEADING_KS
                )
            )
        }

        if (RobotConfig.DATALOG_ENABLED || DrivetrainConfig.DATALOG_ENABLED) {
            val angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES)
            val bestDetection = vision.getBestDetection()
            return DriveTelemetry(
                requestedForward = forward,
                limitedForward = driveForward,
                requestedTurn = turn,
                heading = Math.toDegrees(currentHeadingRad),
                targetHeading = Math.toDegrees(customHeadingRad),
                headingError = Math.toDegrees(
                    minimalAngleDifference(customHeadingRad, currentHeadingRad)
                ),
                headingCorrection = lastCorrectionPower,
                proportionalCorrection = headingController.lastProportional,
                integralCorrection = headingController.lastIntegral,
                derivativeCorrection = headingController.lastDerivative,
                yawRate = angularVelocity.zRotationRate.toDouble(),
                leftTargetVelocity = left * leftMotor.motorType.achieveableMaxTicksPerSecond,
                leftActualVelocity = leftMotor.velocity,
                rightTargetVelocity = right * rightMotor.motorType.achieveableMaxTicksPerSecond,
                rightActualVelocity = rightMotor.velocity,
                leftCurrentAmps = leftMotor.getCurrent(CurrentUnit.AMPS),
                rightCurrentAmps = rightMotor.getCurrent(CurrentUnit.AMPS),
                linearSpeedMmPerSecond = (leftMotor.velocity + rightMotor.velocity) / 2.0 * DrivetrainConfig.millimetersPerEncoderTick,
                batteryVoltage = batteryVoltage,
                headingHoldEnabled = activeHeadingHoldEnabled,
                leftEncoderPosition = leftMotor.currentPosition,
                rightEncoderPosition = rightMotor.currentPosition,
                leftMotorPower = left,
                rightMotorPower = right,
                pitchDegrees = robotAngles.getPitch(AngleUnit.DEGREES),
                rollDegrees = robotAngles.getRoll(AngleUnit.DEGREES),
                pitchRate = angularVelocity.xRotationRate.toDouble(),
                rollRate = angularVelocity.yRotationRate.toDouble(),
                leftWheelSpeedMmPerSecond = leftMotor.velocity * DrivetrainConfig.millimetersPerEncoderTick,
                rightWheelSpeedMmPerSecond = rightMotor.velocity * DrivetrainConfig.millimetersPerEncoderTick,
                tiltingSafety = robotTilting,
                highSpeedTurnBoostActive = false,
                poseX = currentPose.position.x,
                poseY = currentPose.position.y,
                poseHeading = Math.toDegrees(currentPose.heading.toDouble()),
                visionActive = bestDetection != null,
                bestDetectionId = bestDetection?.id ?: -1
            )
        }
        return null
    }

    private fun setMotorPowersSmart(left: Double, right: Double, batteryVoltage: Double) {
        val voltageNorm = if (batteryVoltage > 0.0) {
            (RobotConfig.NOMINAL_BATTERY_VOLTAGE / batteryVoltage)
                .coerceIn(DrivetrainConfig.MIN_VOLTAGE_COMPENSATION, DrivetrainConfig.MAX_VOLTAGE_COMPENSATION)
        } else 1.0
        
        val lp = (left * voltageNorm).coerceIn(-1.0, 1.0)
        val rp = (right * voltageNorm).coerceIn(-1.0, 1.0)

        if (abs(lp - lastLeftPower) > DrivetrainConfig.MOTOR_POWER_TOLERANCE) {
            leftMotor.power = lp
            lastLeftPower = lp
        }
        if (abs(rp - lastRightPower) > DrivetrainConfig.MOTOR_POWER_TOLERANCE) {
            rightMotor.power = rp
            lastRightPower = rp
        }
    }

    private fun setMotorPowersSmart(left: Double, right: Double) {
        val lp = (left).coerceIn(-1.0, 1.0)
        val rp = (right).coerceIn(-1.0, 1.0)

        if (abs(lp - lastLeftPower) > DrivetrainConfig.MOTOR_POWER_TOLERANCE) {
            leftMotor.power = lp
            lastLeftPower = lp
        }
        if (abs(rp - lastRightPower) > DrivetrainConfig.MOTOR_POWER_TOLERANCE) {
            rightMotor.power = rp
            lastRightPower = rp
        }
    }

    fun toggleHeadingHold() {
        activeHeadingHoldEnabled = !activeHeadingHoldEnabled
        if (activeHeadingHoldEnabled) {
            customHeadingRad = imu.robotYawPitchRollAngles.getYaw(AngleUnit.RADIANS)
        }
    }

    fun stop() {
        setMotorPowersSmart(0.0, 0.0)
        activeHeadingHoldEnabled = false
        pendingHeadingLock = false
        headingController.reset()
        vision.stop()
    }

    private fun minimalAngleDifference(target: Double, current: Double): Double {
        var diff = target - current
        while (diff > PI) diff -= 2 * PI
        while (diff < -PI) diff += 2 * PI
        return diff
    }

    fun getPose(): Pose2d = poseEstimator.pose

    fun setPose(pose: Pose2d) {
        localizer.setPose(pose)

        val imuHeading = Rotation2d.exp(
            imu.robotYawPitchRollAngles
                .getYaw(AngleUnit.RADIANS)
        )

        poseEstimator.resetPose(
            pose,
            imuHeading
        )
    }

    fun getCustomHeadingRad(): Double = customHeadingRad

    fun getLeftEncoderPosition(): Int = leftMotor.currentPosition
    fun getRightEncoderPosition(): Int = rightMotor.currentPosition

    fun setCustomHeadingDeg(headingDeg: Double) {
        customHeadingRad = Math.toRadians(headingDeg)
        activeHeadingHoldEnabled = true
        headingController.reset()
    }

    fun resetHeading(targetHeadingDeg: Double = 0.0) {
        imu.resetYaw()
        customHeadingRad = Math.toRadians(targetHeadingDeg)
        activeHeadingHoldEnabled = true
        headingController.reset()
    }


    /**
     * Get comprehensive heading correction data for dashboard telemetry
     * Call this after drive() method to get latest data
     */
    fun getHeadingCorrectionData(): HeadingCorrectionData {
        return HeadingCorrectionData(
            activeHeadingHoldEnabled,
            pendingHeadingLock,
            lastTargetHeadingDeg,
            lastCurrentHeadingDeg,
            lastHeadingErrorDeg,
            lastCorrectionPower,
            lastTiltingSafety,
            Math.toDegrees(getCustomHeadingRad()),
        )
    }

    /**
     * Data class for heading correction telemetry
     */
    class HeadingCorrectionData(
        val isActive: Boolean,
        val isPending: Boolean,
        val targetHeadingDeg: Double,
        val currentHeadingDeg: Double,
        val headingErrorDeg: Double,
        val correctionPower: Double,
        val tiltingSafety: Boolean,
        val customHeadingDeg: Double,
    )
}
