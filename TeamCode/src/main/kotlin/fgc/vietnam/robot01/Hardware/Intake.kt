package fgc.vietnam.robot01.Hardware

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

enum class IntakeState { OFF, INTAKE, OUTTAKE, TRANSFER }

internal class Intake(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotor::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    private val hexMotor = hardwareMap.get(DcMotor::class.java, "transfer").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    private val servoRightBack = hardwareMap.get(CRServo::class.java, "leftBackIntakeServo").apply {
        direction = DcMotorSimple.Direction.FORWARD
    }
    private val servoLeftBack = hardwareMap.get(CRServo::class.java, "rightBackIntakeServo").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    private val servoRightFront = hardwareMap.get(CRServo::class.java, "leftFrontIntakeServo").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val servoLeftFront = hardwareMap.get(CRServo::class.java, "rightFrontIntakeServo").apply {
        direction = DcMotorSimple.Direction.FORWARD
    }


    var intakeEnabled: Boolean = false
        private set

    var outtakeHeld: Boolean = false
        private set

    var hexReverseHeld: Boolean = false
        private set

    var state: IntakeState = IntakeState.INTAKE
    fun intake() {
        motor.power = 1.0
        hexMotor.power = -1.0
        state = IntakeState.INTAKE
    }


    fun outtake() {
        motor.power = -1.0
        hexMotor.power = -1.0
        state = IntakeState.OUTTAKE
    }

    fun transfer() {
        motor.power = 1.0
        hexMotor.power = 1.0
        state = IntakeState.TRANSFER
    }

    fun stopMotor() {
        motor.power = 0.0;
        hexMotor.power = 0.0
        state = IntakeState.OFF
    }

    val motorPower: Double get() = motor.power
    val hexMotorPower: Double get() = hexMotor.power
    val servoRightPower: Double get() = servoRightBack.power
    val servoLeftPower: Double get() = servoLeftBack.power



    fun moveServosForward() {
        servoRightBack.power = 1.0
        servoLeftBack.power = 1.0
        servoRightFront.power = 1.0
        servoLeftFront.power = 1.0
    }

    fun moveServosBackward() {
        servoRightBack.power = -1.0
        servoLeftBack.power = -1.0
        servoRightFront.power = -1.0
        servoLeftFront.power = -1.0
    }

    fun stopServos() {
        servoRightBack.power = 0.0
        servoLeftBack.power = 0.0
        servoRightFront.power = 0.0
        servoLeftFront.power = 0.0
    }
}
