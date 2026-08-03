package fgc.vietnam.robot01

import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap

internal class Intake(hardwareMap: HardwareMap) {
    // Intake motor on Extension Hub Port 0
    private val motor = hardwareMap.get(DcMotor::class.java, "intake").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    // Core Hex motor on Extension Hub Port 1 (2nd motor port)
    private val hexMotor = hardwareMap.get(DcMotor::class.java, "intakeHex").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    // 2 CRServos on Control Hub (mirrored)
    private val servo1 = hardwareMap.get(CRServo::class.java, "servo1").apply {
        direction = DcMotorSimple.Direction.FORWARD
    }
    private val servo2 = hardwareMap.get(CRServo::class.java, "servo2").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    // Toggle: false = hex spins same direction as intake
    //         true  = hex spins opposite direction to intake (default)
    var hexReversed: Boolean = true
        private set

    fun toggleHexDirection() {
        hexReversed = !hexReversed
    }

    // Readable commanded power state (for datalogging)
    val motorPower: Double get() = motor.power
    val hexMotorPower: Double get() = hexMotor.power
    val servo1Power: Double get() = servo1.power
    val servo2Power: Double get() = servo2.power

    fun startIntake() {
        motor.power = 1.0
        hexMotor.power = if (hexReversed) -1.0 else 1.0
    }

    fun startOuttake() {
        motor.power = -1.0
        // Hex motor always spins opposite to the intake (-1.0) during outtake
        hexMotor.power = 1.0
    }

    fun stopMotor() {
        motor.power = 0.0
        hexMotor.power = 0.0
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
