package fgc.vietnam.robot01

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

enum class IntakeState { OFF, INTAKE, OUTTAKE }

internal class Intake(hardwareMap: HardwareMap) {
    private val motor = hardwareMap.get(DcMotor::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    private val hexMotor = hardwareMap.get(DcMotor::class.java, "intakeHex").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    private val servo1 = hardwareMap.get(CRServo::class.java, "servo1").apply {
        direction = DcMotorSimple.Direction.FORWARD
    }
    private val servo2 = hardwareMap.get(CRServo::class.java, "servo2").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    var intakeEnabled: Boolean = false
        private set

    var outtakeHeld: Boolean = false
        private set

    var hexReverseHeld: Boolean = false
        private set

    val state: IntakeState
        get() = when {
            outtakeHeld -> IntakeState.OUTTAKE
            intakeEnabled -> IntakeState.INTAKE
            else -> IntakeState.OFF
        }

    var hexReversed: Boolean = true
        private set

    fun toggleHexDirection() {
        hexReversed = !hexReversed
        applyState()
    }

    fun toggleIntake() {
        intakeEnabled = !intakeEnabled
        applyState()
    }

    fun setOuttakeHeld(held: Boolean) {
        outtakeHeld = held
        applyState()
    }

    fun setHexReverseHeld(held: Boolean) {
        hexReverseHeld = held
        applyState()
    }

    val motorPower: Double get() = motor.power
    val hexMotorPower: Double get() = hexMotor.power
    val servo1Power: Double get() = servo1.power
    val servo2Power: Double get() = servo2.power

    fun startIntake() {
        intakeEnabled = true
        outtakeHeld = false
        applyState()
    }

    fun startOuttake() {
        outtakeHeld = true
        applyState()
    }

    fun stopMotor() {
        intakeEnabled = false
        outtakeHeld = false
        hexReverseHeld = false
        applyState()
    }

    private fun applyState() {
        val hexTargetPower = if (hexReverseHeld) 1.0 else -1.0
        when {
            outtakeHeld -> {
                motor.power = -1.0
                hexMotor.power = hexTargetPower
            }
            intakeEnabled -> {
                motor.power = 1.0
                hexMotor.power = hexTargetPower
            }
            else -> {
                motor.power = 0.0
                hexMotor.power = 0.0
            }
        }
    }

    fun moveServosForward() {
        servo1.power = 1.0
        servo2.power = 1.0
    }

    fun moveServosBackward() {
        servo1.power = -1.0
        servo2.power = -1.0
    }

    fun stopServos() {
        servo1.power = 0.0
        servo2.power = 0.0
    }
}
