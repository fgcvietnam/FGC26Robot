package fgc.vietnam.robot01.Hardware

import TeamVietnam.control.PIDFController
import com.qualcomm.hardware.rev.Rev2mDistanceSensor
import com.qualcomm.robotcore.hardware.CRServo
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DistanceSensor
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.util.ElapsedTime
import fgc.vietnam.robot01.Config.ClimbConfig
import fgc.vietnam.robot01.Utils.FeedForward
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit
import kotlin.math.abs

internal class Climb(hardwareMap: HardwareMap) {

    enum class ClimbState { CLIMBING, STOPPED, RETRACTING }

    private val holdTimer = ElapsedTime()
    private var shouldHold = false

    private val motorAbove = hardwareMap.get(DcMotorEx::class.java, "climbAbove").apply {
        direction = DcMotorSimple.Direction.FORWARD
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


    fun climbForward(inputPower: Double) {
        motorAbove.power = 1.0 * inputPower
        motorBelow.power = 1.0 * inputPower

        shouldHold = true
        holdTimer.reset()

        state = ClimbState.CLIMBING
    }


    fun climbBackward(inputPower: Double) {
        motorAbove.power = -1.0 * inputPower
        motorBelow.power = -1.0 * inputPower
        state = ClimbState.RETRACTING
    }

    fun stop() {
        if (shouldHold && holdTimer.milliseconds() >= ClimbConfig.HOLD_DELAY_CONFIRM_MS) {
            motorAbove.power = ClimbConfig.HOLD_STOP_POWER
            motorBelow.power = ClimbConfig.HOLD_STOP_POWER
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

    fun toggle() {
        if (state != ClimbState.CLIMBING) {
            motorAbove.power = 1.0
            motorBelow.power = 1.0
            state = ClimbState.CLIMBING
        } else if (state == ClimbState.CLIMBING) {
            if (motorAbove.power != ClimbConfig.HOLD_STOP_POWER) motorAbove.power = ClimbConfig.HOLD_STOP_POWER
            if (motorBelow.power != ClimbConfig.HOLD_STOP_POWER) motorBelow.power = ClimbConfig.HOLD_STOP_POWER
            state = ClimbState.RETRACTING
        }
    }

    fun getMotorAbovePower(): Double = motorAbove.power
    fun getMotorBelowPower(): Double = motorBelow.power
    fun getClimbState(): String = state.name
    fun getServoPower(): Double = servoClimb.power
}