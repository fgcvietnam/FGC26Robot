import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import FGC.Vietnam.Config.ClimbConfig
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit

internal class Climb(hardwareMap: HardwareMap) {

    enum class ClimbState { CLIMBING, STOPPED, RETRACTING }

    private val holdPowerTimer = ElapsedTime()
    private var shouldHold = false
    private var holdPower = 0.0

    private val holdInputTimer = ElapsedTime()
    private var previousHoldInput = false

    private val motorAbove = hardwareMap.get(DcMotorEx::class.java, "climbAbove").apply {
        direction = DcMotorSimple.Direction.REVERSE
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        power = 0.0
    }

    private val motorBelow = hardwareMap.get(DcMotorEx::class.java, "climbBelow").apply {
        direction = DcMotorSimple.Direction.FORWARD
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.BRAKE
        mode = DcMotor.RunMode.RUN_USING_ENCODER
        power = 0.0
    }

    private val servoClimb = hardwareMap.get(CRServo::class.java, "climbServo").apply {
        direction = DcMotorSimple.Direction.REVERSE
    }

    private val distanceSensor = hardwareMap.get(Rev2mDistanceSensor::class.java, "climbDistanceSensor")

    private var state: ClimbState = ClimbState.STOPPED


    fun climbForward(inputPower: Float) {
        motorAbove.power = 1.0 * inputPower
        motorBelow.power = 1.0 * inputPower

        shouldHold = true
        holdPowerTimer.reset()

        state = ClimbState.CLIMBING
    }


    fun climbBackward(inputPower: Double) {
        motorAbove.power = -1.0 * inputPower
        motorBelow.power = -1.0 * inputPower
        state = ClimbState.RETRACTING
    }

    fun stop() {
        if (shouldHold && holdPower >= ClimbConfig.HOLD_STOP_POWER) {
            motorAbove.power = holdPower
            motorBelow.power = holdPower
        } else {
            motorAbove.power = 0.0
            motorBelow.power = 0.0
        }

        state = ClimbState.STOPPED
    }

    fun climbExtend(){
        if (servoClimb.power == 1.0) return
        if (distanceSensor.getDistance(DistanceUnit.MM) > ClimbConfig.CLIMB_EXTEND_DISTANCE_MM) climbExtendStop()
        servoClimb.power = 1.0
    }

    fun climbRetract(){
        servoClimb.power = -1.0
    }

    fun climbExtendStop(){
        if (servoClimb.power == 0.0) return
        servoClimb.power = 0.0
    }

    fun getDistance(): Double = distanceSensor.getDistance(DistanceUnit.MM)

    fun getHoldPower(holdTimeSeconds: Double): Double {
        return (
                ClimbConfig.HOLD_STOP_POWER +
                        holdTimeSeconds * ClimbConfig.HOLD_POWER_INCREASE_PER_SECOND
                ).coerceAtMost(ClimbConfig.MAX_HOLD_POWER)
    }

    fun updateHoldPower(leftTrigger: Float) {
        val pressed = leftTrigger > ClimbConfig.TRIGGER_DEADBAND

        if (pressed && !previousHoldInput) {
            holdInputTimer.reset()
        }

        if (pressed) {
            holdPower = getHoldPower(holdInputTimer.seconds())

            motorAbove.power = holdPower
            motorBelow.power = holdPower
        }

        previousHoldInput = pressed
    }


    fun getMotorAbovePower(): Double = motorAbove.power
    fun getMotorBelowPower(): Double = motorBelow.power
    fun getClimbState(): String = state.name
    fun getServoPower(): Double = servoClimb.power
}