package fgc.vietnam.robot01.Hardware

import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import FGC.Vietnam.Config.IntakeConfig
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

enum class IntakeState { OFF, INTAKE, OUTTAKE, TRANSFER}
enum class IntakeSlidePosition { HOME, EXTENDED, EXTENDING, RETRACTING, UNKNOWN }

internal data class IntakeTelemetry(
    val state: IntakeState,
    val leftSlidePosition: IntakeSlidePosition,
    val rightSlidePosition: IntakeSlidePosition,
    val slidesSynchronized: Boolean,
    val slideSkewState: String,
    val motorPower: Double,
    val hexMotorPower: Double,
    val servoLeftBelowPower: Double,
    val servoRightBelowPower: Double,
    val servoLeftAbovePower: Double,
    val servoRightAbovePower: Double,
    val leftLimitSwitchPressed: Boolean,
    val rightLimitSwitchPressed: Boolean,
    val bothLimitsPressed: Boolean,
    val limitSyncState: String,
    val leftMagneticSwitchPressed: Boolean,
    val rightMagneticSwitchPressed: Boolean,
    val bothMagnetsDetected: Boolean,
    val magnetSyncState: String,
    val leftMagnetConfirmed: Boolean,
    val rightMagnetConfirmed: Boolean,
    val bothMagnetsConfirmed: Boolean,
    val intakeJamConfirmed: Boolean,
    val transferJamConfirmed: Boolean,
    val isUnjamming: Boolean,
    val motorCurrentAmps: Double,
    val hexMotorCurrentAmps: Double,
)

internal class Intake(hardwareMap: HardwareMap) {

    private var leftIntakeSlidePosition: IntakeSlidePosition = IntakeSlidePosition.UNKNOWN

    private var rightIntakeSlidePosition: IntakeSlidePosition = IntakeSlidePosition.UNKNOWN

    private val motor = hardwareMap.get(DcMotorEx::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        setCurrentAlert(5.0, CurrentUnit.AMPS)
        power = 0.0
    }

    private val hexMotor = hardwareMap.get(DcMotorEx::class.java, "transfer").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        setCurrentAlert(5.0, CurrentUnit.AMPS)
        power = 0.0
    }

    private val servoLeftBelow = hardwareMap.get(CRServo::class.java, "leftBelowIntakeServo").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val servoRightBelow = hardwareMap.get(CRServo::class.java, "rightBelowIntakeServo").apply {
        direction = DcMotorSimple.Direction.FORWARD
    }

    private val servoLeftAbove = hardwareMap.get(CRServo::class.java, "leftAboveIntakeServo").apply {
        direction = DcMotorSimple.Direction.FORWARD
    }
    private val servoRightAbove = hardwareMap.get(CRServo::class.java, "rightAboveIntakeServo").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    private val leftLimitSwitch = hardwareMap.get(RevTouchSensor::class.java, "leftLimitSwitchExtension")
    private val rightLimitSwitch = hardwareMap.get(RevTouchSensor::class.java, "rightLimitSwitchExtension")

    private val rightMagneticSwitch = hardwareMap.get(RevTouchSensor::class.java, "rightMagneticSwitchExtension")
    private val leftMagneticSwitch = hardwareMap.get(RevTouchSensor::class.java, "leftMagneticSwitchExtension")
    private var state: IntakeState = IntakeState.OFF

    val motorPower: Double get() = motor.power
    val hexMotorPower: Double get() = hexMotor.power
    val servoRightBelowPower: Double get() = servoRightBelow.power
    val servoLeftBelowPower: Double get() = servoLeftBelow.power
    val servoRightAbovePower: Double get() = servoRightAbove.power
    val servoLeftAbovePower: Double get() = servoLeftAbove.power
    private val leftMagnetTimer = ElapsedTime()
    private val rightMagnetTimer = ElapsedTime()
    private var leftMagnetTiming = false
    private var rightMagnetTiming = false
    private var intakeJamTimer = ElapsedTime()
    private var transferJamTimer = ElapsedTime()
    private var unjamTimer = ElapsedTime()
    private var intakeJamTiming = false
    private var transferJamTiming = false

    private var unjamming = false

    fun intake() {
        motor.power = 1.0
        hexMotor.power = 0.0
        state = IntakeState.INTAKE
    }


    fun outtake() {
        motor.power = -1.0
        hexMotor.power = -1.0
        state = IntakeState.OUTTAKE
    }

    fun transfer(shooterReady: Boolean) {
        if (shooterReady) {
            hexMotor.power = 1.0
        } else {
            hexMotor.power = 0.0
        }
        state = IntakeState.TRANSFER
    }

    fun stopMotor() {
        motor.power = 0.0;
        hexMotor.power = 0.0
        state = IntakeState.OFF
    }

    fun toggle(){
        if (state == IntakeState.OFF) {
            motor.power = 1.0
            hexMotor.power = 1.0
            state = IntakeState.INTAKE
        } else if (state != IntakeState.OFF) {
            motor.power = 0.0
            hexMotor.power = 0.0
            state = IntakeState.OFF
        }
    }

    fun update() {
        if (leftMagneticSwitch.isPressed) {
            if (!leftMagnetTiming) {
                leftMagnetTimer.reset()
                leftMagnetTiming = true
            }
        } else {
            leftMagnetTiming = false
        }

        if (rightMagneticSwitch.isPressed) {
            if (!rightMagnetTiming) {
                rightMagnetTimer.reset()
                rightMagnetTiming = true
            }
        } else {
            rightMagnetTiming = false
        }

        if (motor.isOverCurrent) {
            if (!intakeJamTiming) {
                intakeJamTimer.reset()
                intakeJamTiming = true
            }
        } else {
            intakeJamTiming = false
        }

        if (hexMotor.isOverCurrent) {
            if (!transferJamTiming) {
                transferJamTimer.reset()
                transferJamTiming = true
            }
        } else {
            transferJamTiming = false
        }

//        if (intakeJamConfirm() && !unjamming) {
//            unjamming = true
//            unjamTimer.reset()
//            motor.power = 0.0
//        }
//
//        if (unjamTimer.milliseconds() >= IntakeConfig.INTAKE_UNJAM_DELAY_MS){
//            unjamming = false
//        }
//
//        if (unjamming) {
//            if (unjamTimer.milliseconds() < IntakeConfig.INTAKE_UNJAM_DELAY_MS) {
//                motor.power = 0.0      // or reverse
//            } else {
//                unjamming = false
//            }
//        }
    }


    fun moveServosForward() {
        if (rightMagnetConfirmed()) {
            servoRightBelow.power = 0.0
            servoRightAbove.power = 0.0
            rightIntakeSlidePosition = IntakeSlidePosition.EXTENDED;
        } else {
            servoRightBelow.power = 1.0
            servoRightAbove.power = 1.0
            rightIntakeSlidePosition = IntakeSlidePosition.EXTENDING;
        }

        if (leftMagnetConfirmed()) {
            servoLeftBelow.power = 0.0
            servoLeftAbove.power = 0.0
            leftIntakeSlidePosition = IntakeSlidePosition.EXTENDED;
        } else {
            servoLeftBelow.power = 1.0
            servoLeftAbove.power = 1.0
            leftIntakeSlidePosition = IntakeSlidePosition.EXTENDING;
        }

    }

    fun moveServosBackward() {
        if (rightLimitSwitch.isPressed) {
            servoRightBelow.power = 0.0
            servoRightAbove.power = 0.0
            rightIntakeSlidePosition = IntakeSlidePosition.HOME;
        } else {
            servoRightBelow.power = -1.0
            servoRightAbove.power = -1.0
            rightIntakeSlidePosition = IntakeSlidePosition.RETRACTING;
        }
        if (leftLimitSwitch.isPressed) {
            servoLeftBelow.power = 0.0
            servoLeftAbove.power = 0.0
            leftIntakeSlidePosition = IntakeSlidePosition.HOME;
        } else {
            servoLeftBelow.power = -1.0
            servoLeftAbove.power = -1.0
            leftIntakeSlidePosition = IntakeSlidePosition.RETRACTING;
        }
    }

    fun stopServos() {
        servoLeftBelow.power = 0.0
        servoRightBelow.power = 0.0
        servoLeftAbove.power = 0.0
        servoRightAbove.power = 0.0
    }

    fun rightLimitSwitchIsPressed(): Boolean {
        return rightLimitSwitch.isPressed
    }

    fun leftLimitSwitchIsPressed(): Boolean {
        return leftLimitSwitch.isPressed
    }

    fun getIntakeState(): IntakeState {
        return state
    }

    fun getLeftIntakeSlidePosition(): IntakeSlidePosition {
        return leftIntakeSlidePosition
    }
    fun getRightIntakeSlidePosition(): IntakeSlidePosition {
        return rightIntakeSlidePosition
    }

    fun getHexMotor(): DcMotorEx {
        return hexMotor
    }
    fun leftMagnetConfirmed(): Boolean {
        return leftMagnetTiming &&
                leftMagnetTimer.milliseconds() >= IntakeConfig.MAGNETIC_SWITCH_CONFIRM_DELAY_MS
    }

    fun rightMagnetConfirmed(): Boolean {
        return rightMagnetTiming &&
                rightMagnetTimer.milliseconds() >= IntakeConfig.MAGNETIC_SWITCH_CONFIRM_DELAY_MS
    }

    fun leftMagnetRegistered(): Boolean {
        return leftMagneticSwitch.isPressed
    }

    fun rightMagnetRegistered(): Boolean {
        return rightMagneticSwitch.isPressed
    }


    fun bothMagnetsRegistered(): Boolean =
        leftMagneticSwitch.isPressed && rightMagneticSwitch.isPressed

    fun bothMagnetsConfirmed(): Boolean =
        leftMagnetConfirmed() && rightMagnetConfirmed()

    fun getMagnetSyncState(): String = when {
        leftMagneticSwitch.isPressed && rightMagneticSwitch.isPressed -> "BOTH_DETECTED"
        leftMagneticSwitch.isPressed && !rightMagneticSwitch.isPressed -> "LEFT_ONLY_ASYNC"
        !leftMagneticSwitch.isPressed && rightMagneticSwitch.isPressed -> "RIGHT_ONLY_ASYNC"
        else -> "NONE"
    }

    fun bothLimitsPressed(): Boolean =
        leftLimitSwitch.isPressed && rightLimitSwitch.isPressed

    fun getLimitSyncState(): String = when {
        leftLimitSwitch.isPressed && rightLimitSwitch.isPressed -> "BOTH_HOME"
        leftLimitSwitch.isPressed && !rightLimitSwitch.isPressed -> "LEFT_HOME_ASYNC"
        !leftLimitSwitch.isPressed && rightLimitSwitch.isPressed -> "RIGHT_HOME_ASYNC"
        else -> "NONE"
    }

    fun areSlidesSynchronized(): Boolean =
        leftIntakeSlidePosition == rightIntakeSlidePosition

    fun getSlideSkewState(): String =
        if (leftIntakeSlidePosition == rightIntakeSlidePosition) {
            leftIntakeSlidePosition.name
        } else {
            "${leftIntakeSlidePosition.name}_VS_${rightIntakeSlidePosition.name}"
        }

    fun intakeJamConfirm(): Boolean {
        return intakeJamTiming &&
                intakeJamTimer.milliseconds() >= IntakeConfig.INTAKE_JAM_DETECTION_DELAY_MS
    }

    fun transferJamConfirmed(): Boolean {
        return transferJamTiming &&
                transferJamTimer.milliseconds() >= IntakeConfig.INTAKE_JAM_DETECTION_DELAY_MS
    }

    fun isUnjamming() = unjamming
    
    fun getTelemetry(): IntakeTelemetry {
        return IntakeTelemetry(
            state = state,
            leftSlidePosition = leftIntakeSlidePosition,
            rightSlidePosition = rightIntakeSlidePosition,
            slidesSynchronized = areSlidesSynchronized(),
            slideSkewState = getSlideSkewState(),
            motorPower = motor.power,
            hexMotorPower = hexMotor.power,
            servoLeftBelowPower = servoLeftBelow.power,
            servoRightBelowPower = servoRightBelow.power,
            servoLeftAbovePower = servoLeftAbove.power,
            servoRightAbovePower = servoRightAbove.power,
            leftLimitSwitchPressed = leftLimitSwitch.isPressed,
            rightLimitSwitchPressed = rightLimitSwitch.isPressed,
            bothLimitsPressed = bothLimitsPressed(),
            limitSyncState = getLimitSyncState(),
            leftMagneticSwitchPressed = leftMagneticSwitch.isPressed,
            rightMagneticSwitchPressed = rightMagneticSwitch.isPressed,
            bothMagnetsDetected = bothMagnetsRegistered(),
            magnetSyncState = getMagnetSyncState(),
            leftMagnetConfirmed = leftMagnetConfirmed(),
            rightMagnetConfirmed = rightMagnetConfirmed(),
            bothMagnetsConfirmed = bothMagnetsConfirmed(),
            intakeJamConfirmed = intakeJamConfirm(),
            transferJamConfirmed = transferJamConfirmed(),
            isUnjamming = unjamming,
            motorCurrentAmps = motor.getCurrent(CurrentUnit.AMPS),
            hexMotorCurrentAmps = hexMotor.getCurrent(CurrentUnit.AMPS),
        )
    }

}
