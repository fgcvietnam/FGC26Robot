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

    private val servoRight = hardwareMap.get(CRServo::class.java, "leftIntakeServo").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }
    private val servoLeft = hardwareMap.get(CRServo::class.java, "rightIntakeServo").apply {
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
    val servoRightPower: Double get() = servoRight.power
    val servoLeftPower: Double get() = servoLeft.power



    fun moveServosForward() {
        servoRight.power = 1.0
        servoLeft.power = 1.0
    }

    fun moveServosBackward() {
        servoRight.power = -1.0
        servoLeft.power = -1.0
    }

    fun stopServos() {
        servoRight.power = 0.0
        servoLeft.power = 0.0
    }
}
