package fgc.vietnam.robot01.Hardware

import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.HardwareMap
import FGC.Vietnam.Config.RobotConfig
import FGC.Vietnam.Config.FlywheelConfig
import FGC.Vietnam.Config.FlywheelConfig.ENABLE_PIDF_TUNING
import FGC.Vietnam.Config.FlywheelConfig.FF_KA
import FGC.Vietnam.Config.FlywheelConfig.FF_KS
import FGC.Vietnam.Config.FlywheelConfig.FF_KV
import FGC.Vietnam.Config.FlywheelConfig.PIDF_D
import FGC.Vietnam.Config.FlywheelConfig.PIDF_I
import FGC.Vietnam.Config.FlywheelConfig.PIDF_P
import FGC.Vietnam.Config.RobotConfig.DATALOG_ENABLED
import FGC.Vietnam.Utils.FeedForward
import FGC.Vietnam.Utils.PIDFController
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit
import kotlin.math.abs
import kotlin.math.min

internal data class FlywheelMotorTelemetry(
    val rpm: Double,
    val velocity: Double,
    val currentAmps: Double,
    val encoderPosition: Int,
    val motorPower: Double,
)

internal data class FlywheelTelemetry(
    val enabled: Boolean,
    val atSpeed: Boolean,
    val targetRpm: Double,
    val targetVelocity: Double,
    val rightShooterMotor: FlywheelMotorTelemetry,
    val leftShooterMotor: FlywheelMotorTelemetry,
    val shaftRpm: Double,
    val rpmDifference: Double,
    val surfaceSpeedMetersPerSecond: Double,
)


internal class Flywheel(hardwareMap: HardwareMap) {
    private val rightShooterMotor = motor(
        hardwareMap = hardwareMap,
        name = FlywheelConfig.LEFT_SHOOTER_MOTOR,
        direction = DcMotorSimple.Direction.REVERSE,
    )

    private val leftShooterMotor = motor(
        hardwareMap = hardwareMap,
        name = FlywheelConfig.RIGHT_SHOOTER_MOTOR,
        direction = DcMotorSimple.Direction.FORWARD,
    )
    private var controlVelocity = 0.0
    private var mismatchStartTime = -1L
    private var leftEncoderHealthy = true
    private var rightEncoderHealthy = true

    private var enabled = false
    private var targetVelocity = FlywheelConfig.DEFAULT_RPM

    fun toggle() {
        enabled = !enabled
    }

    fun enable() {
        enabled = true
    }

    fun disable() {
        enabled = false
    }


    val shooterPIDF = PIDFController(
        kP = PIDF_P,
        kI = PIDF_I,
        kD = PIDF_D,
        FeedForward(FF_KS, FF_KV)
    )

    fun update(batteryVoltage: Double): FlywheelTelemetry? {
        if (enabled) {
            val now = System.currentTimeMillis() / 1000.0
            targetVelocity = FlywheelConfig.DEFAULT_RPM


            if (ENABLE_PIDF_TUNING) {
                shooterPIDF.setPIDF(
                    PIDF_P,
                    PIDF_I,
                    PIDF_D,
                    FeedForward(
                        FF_KS,
                        FF_KV,
                        FF_KA
                    )
                )
            }


            val leftVelocity: Double = leftShooterMotor.velocity
            val rightVelocity: Double = rightShooterMotor.velocity


            val velocityDifference = abs(leftVelocity - rightVelocity)

            val allowedDifference = maxOf(
                FlywheelConfig.ENCODER_MIN_VELOCITY,
                maxOf(leftVelocity, rightVelocity) * FlywheelConfig.ENCODER_MISMATCH_RATIO
            )

            if (velocityDifference > allowedDifference) {
                if (mismatchStartTime == -1L) {
                    mismatchStartTime = System.currentTimeMillis()
                }
                if (System.currentTimeMillis() - mismatchStartTime >
                    FlywheelConfig.ENCODER_MISMATCH_TIME_MS
                ) {
                    leftEncoderHealthy =
                        !(leftVelocity < FlywheelConfig.ENCODER_MIN_VELOCITY &&
                                rightVelocity > FlywheelConfig.ENCODER_MIN_VELOCITY * 5)

                    rightEncoderHealthy =
                        !(rightVelocity < FlywheelConfig.ENCODER_MIN_VELOCITY &&
                                leftVelocity > FlywheelConfig.ENCODER_MIN_VELOCITY * 5)
                }

            } else {
                mismatchStartTime = -1L
                leftEncoderHealthy = true
                rightEncoderHealthy = true
            }

            controlVelocity = when {
                leftEncoderHealthy -> leftVelocity
                rightEncoderHealthy -> rightVelocity
                else -> 0.0
            }

            val output = shooterPIDF.calculate(
                abs(controlVelocity),
                targetVelocity
            )

            var voltageNorm: Double = RobotConfig.NOMINAL_BATTERY_VOLTAGE / batteryVoltage
            voltageNorm = min(1.0, voltageNorm)

            leftShooterMotor.power = output * voltageNorm
            rightShooterMotor.power = output * voltageNorm


            if (DATALOG_ENABLED) {
                val leftState = motorTelemetry(leftShooterMotor)
                val rightState = motorTelemetry(rightShooterMotor)

                val shaftRpm =
                    (leftState.rpm + rightState.rpm) / 2.0

                val allowedErrorRpm = maxOf(
                    FlywheelConfig.MIN_READY_ERROR_RPM,
                    targetVelocity * FlywheelConfig.READY_ERROR_RATIO
                )

                return FlywheelTelemetry(
                    enabled = true,
                    atSpeed =
                        abs(leftState.rpm - targetVelocity) <= allowedErrorRpm &&
                                abs(rightState.rpm - targetVelocity) <= allowedErrorRpm,

                    targetRpm = targetVelocity,
                    targetVelocity = FlywheelConfig.rpmToTicksPerSecond(targetVelocity),

                    leftShooterMotor = leftState,
                    rightShooterMotor = rightState,

                    shaftRpm = shaftRpm,
                    rpmDifference = abs(leftState.rpm - rightState.rpm),

                    surfaceSpeedMetersPerSecond =
                        FlywheelConfig.rpmToSurfaceSpeedMetersPerSecond(shaftRpm)
                )
            } else return null;



        } else {
            leftShooterMotor.velocity = 0.0
            rightShooterMotor.velocity = 0.0
            leftShooterMotor.power = 0.0
            rightShooterMotor.power = 0.0
            return null;
        }

    }

    fun stop() {
        enabled = false
        leftShooterMotor.velocity = 0.0
        rightShooterMotor.velocity = 0.0
    }

    private fun motorTelemetry(motor: DcMotorEx): FlywheelMotorTelemetry =
        FlywheelMotorTelemetry(
            rpm = FlywheelConfig.ticksPerSecondToRpm(motor.velocity),
            velocity = motor.velocity,
            currentAmps = motor.getCurrent(CurrentUnit.AMPS),
            encoderPosition = motor.currentPosition,
            motorPower = motor.power,
        )

    private fun motor(
        hardwareMap: HardwareMap,
        name: String,
        direction: DcMotorSimple.Direction,
    ): DcMotorEx = hardwareMap.get(DcMotorEx::class.java, name).apply {
        this.direction = direction
        zeroPowerBehavior = DcMotor.ZeroPowerBehavior.FLOAT
        mode = DcMotor.RunMode.STOP_AND_RESET_ENCODER
        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER
        power = 0.0
    }

    private fun getVelocity(motor: DcMotorEx): Double = motor.velocity

    fun isEnable(): Boolean = enabled;

    fun getTargetVelocity(): Double = targetVelocity

    fun getCurrentVelocity(): Double = controlVelocity

    fun getPower(): Double = leftShooterMotor.power

    fun atTargetVelocity(): Boolean = abs(controlVelocity - targetVelocity) <= maxOf(
        FlywheelConfig.MIN_READY_ERROR_RPM,
        targetVelocity * FlywheelConfig.READY_ERROR_RATIO
    )

    fun getLeftShooterMotor(): DcMotorEx = leftShooterMotor
    fun getRightShooterMotor(): DcMotorEx = rightShooterMotor


}
