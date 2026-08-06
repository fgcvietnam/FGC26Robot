package fgc.vietnam.robot01.Hardware

import com.qualcomm.hardware.rev.RevTouchSensor
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit

enum class ClimbState { OFF, INTAKE, OUTTAKE, TRANSFER }
enum class IntakeSlidePosition { HOME, EXTENDED, EXTENDING, UNKNOWN }

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
        direction = DcMotorSimple.Direction.FORWARD
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
    private var state: ClimbState = ClimbState.INTAKE

    val motorPower: Double get() = motor.power
    val hexMotorPower: Double get() = hexMotor.power
    val servoRightBelowPower: Double get() = servoRightBelow.power
    val servoLeftBelowPower: Double get() = servoLeftBelow.power
    val servoRightAbovePower: Double get() = servoRightAbove.power
    val servoLeftAbovePower: Double get() = servoLeftAbove.power
    fun intake() {
        motor.power = 1.0
        hexMotor.power = 0.0
        state = ClimbState.INTAKE
    }


    fun outtake() {
        motor.power = -1.0
        hexMotor.power = -1.0
        state = ClimbState.OUTTAKE
    }

    fun transfer(shooterReady: Boolean) {
        if (shooterReady) {
            hexMotor.power = 1.0
        } else {
            hexMotor.power = 0.0
        }
        state = ClimbState.TRANSFER
    }

    fun stopMotor() {
        motor.power = 0.0;
        hexMotor.power = 0.0
        state = ClimbState.OFF
    }

    fun toggle(){
        if (state == ClimbState.OFF) {
            motor.power = 1.0
            hexMotor.power = 1.0
            state = ClimbState.INTAKE
        } else {
            motor.power = 0.0
            hexMotor.power = 0.0
            state = ClimbState.OFF
        }
    }


    fun moveServosForward() {
        servoLeftBelow.power = 1.0
        servoRightBelow.power = 1.0
        servoLeftAbove.power = 1.0
        servoRightAbove.power = 1.0
        leftIntakeSlidePosition = IntakeSlidePosition.EXTENDING;
        rightIntakeSlidePosition = IntakeSlidePosition.EXTENDING;

    }

    fun moveServosBackward() {
        if (rightLimitSwitch.isPressed) {
            servoRightBelow.power = 0.0
            servoRightAbove.power = 0.0
            leftIntakeSlidePosition = IntakeSlidePosition.HOME;
        } else {
            servoRightBelow.power = -1.0
            servoRightAbove.power = -1.0
            leftIntakeSlidePosition = IntakeSlidePosition.UNKNOWN;

        }

        if (leftLimitSwitch.isPressed) {
            servoLeftBelow.power = 0.0
            servoLeftAbove.power = 0.0
            rightIntakeSlidePosition = IntakeSlidePosition.HOME;
        } else {
            servoLeftBelow.power = -1.0
            servoLeftAbove.power = -1.0
            rightIntakeSlidePosition = IntakeSlidePosition.UNKNOWN;
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

    fun getIntakeState(): ClimbState {
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
}
