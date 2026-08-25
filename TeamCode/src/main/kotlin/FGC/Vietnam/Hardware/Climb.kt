package fgc.vietnam.robot01.Hardware

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
    fun getMotorAbovePosition(): Int = motorAbove.currentPosition
    fun getMotorBelowPosition(): Int = motorBelow.currentPosition
    fun getMotorAboveVelocity(): Double = motorAbove.velocity
    fun getMotorBelowVelocity(): Double = motorBelow.velocity
    fun getClimbState(): String = state.name
    fun getServoPower(): Double = servoClimb.power
    fun isShouldHold(): Boolean = shouldHold
    fun getCurrentHoldPower(): Double = holdPower

    fun getTelemetry(batteryVoltage: Double = 12.0): ClimbTelemetry {
        val currentAbove = try { motorAbove.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) } catch (_: Exception) { 0.0 }
        val currentBelow = try { motorBelow.getCurrent(org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit.AMPS) } catch (_: Exception) { 0.0 }
        val velAbove = motorAbove.velocity
        val velBelow = motorBelow.velocity
        val posAbove = motorAbove.currentPosition
        val posBelow = motorBelow.currentPosition
        val motorAboveRpm = velAbove * 60.0 / ClimbConfig.MOTOR_ENCODER_TICKS_PER_REVOLUTION
        val motorBelowRpm = velBelow * 60.0 / ClimbConfig.MOTOR_ENCODER_TICKS_PER_REVOLUTION
        val shaftAboveRpm = velAbove * 60.0 / ClimbConfig.encoderTicksPerShaftRevolution
        val shaftBelowRpm = velBelow * 60.0 / ClimbConfig.encoderTicksPerShaftRevolution
        val distMm = try { distanceSensor.getDistance(DistanceUnit.MM) } catch (_: Exception) { Double.NaN }

        val pwrAboveWatts = currentAbove * batteryVoltage * kotlin.math.abs(motorAbove.power)
        val pwrBelowWatts = currentBelow * batteryVoltage * kotlin.math.abs(motorBelow.power)

        return ClimbTelemetry(
            motorAbovePower = motorAbove.power,
            motorBelowPower = motorBelow.power,
            motorAbovePositionTicks = posAbove,
            motorBelowPositionTicks = posBelow,
            encoderPositionDiffTicks = posAbove - posBelow,
            motorAboveVelocityTicksPerSec = velAbove,
            motorBelowVelocityTicksPerSec = velBelow,
            motorVelocityDiffTicksPerSec = velAbove - velBelow,
            motorAboveRpm = motorAboveRpm,
            motorBelowRpm = motorBelowRpm,
            shaftAboveRpm = shaftAboveRpm,
            shaftBelowRpm = shaftBelowRpm,
            motorAboveCurrentAmps = currentAbove,
            motorBelowCurrentAmps = currentBelow,
            totalCurrentAmps = currentAbove + currentBelow,
            currentDiffAmps = currentAbove - currentBelow,
            motorAbovePowerWatts = pwrAboveWatts,
            motorBelowPowerWatts = pwrBelowWatts,
            totalPowerWatts = pwrAboveWatts + pwrBelowWatts,
            servoPower = servoClimb.power,
            distanceMm = distMm,
            climbState = state.name,
            shouldHold = shouldHold,
            holdPower = holdPower
        )
    }
}

internal data class ClimbTelemetry(
    val motorAbovePower: Double,
    val motorBelowPower: Double,
    val motorAbovePositionTicks: Int,
    val motorBelowPositionTicks: Int,
    val encoderPositionDiffTicks: Int,
    val motorAboveVelocityTicksPerSec: Double,
    val motorBelowVelocityTicksPerSec: Double,
    val motorVelocityDiffTicksPerSec: Double,
    val motorAboveRpm: Double,
    val motorBelowRpm: Double,
    val shaftAboveRpm: Double,
    val shaftBelowRpm: Double,
    val motorAboveCurrentAmps: Double,
    val motorBelowCurrentAmps: Double,
    val totalCurrentAmps: Double,
    val currentDiffAmps: Double,
    val motorAbovePowerWatts: Double,
    val motorBelowPowerWatts: Double,
    val totalPowerWatts: Double,
    val servoPower: Double,
    val distanceMm: Double,
    val climbState: String,
    val shouldHold: Boolean,
    val holdPower: Double
)