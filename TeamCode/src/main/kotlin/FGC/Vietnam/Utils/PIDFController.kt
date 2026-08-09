package FGC.Vietnam.Utils

import com.qualcomm.robotcore.util.ElapsedTime
import com.qualcomm.robotcore.util.Range
import java.util.Locale
import kotlin.concurrent.Volatile
import kotlin.math.abs

/**
 * Advanced PIDF Controller with comprehensive feed-forward control
 *
 * Features:
 * - Full feed-forward implementation (kS, kV, kA)
 * - PIDF components with integral windup protection
 * - Derivative kick prevention
 * - Configurable output limits and deadband
 * - Thread-safe operation for FTC environment
 * - Telemetry support for tuning and debugging
 *
 * Primary use: Flywheel velocity control
 * Secondary use: Heading control (if needed)
 */
class PIDFController(kP: Double, kI: Double, kD: Double, var feedForward: FeedForward) {
    // Gain getters
    // PIDF Gains
    @get:Synchronized
    @Volatile
    var kP: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var kI: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var kD: Double = 0.0
        private set

    // Control limits
    @Volatile
    private var outputMin = -1.0

    @Volatile
    private var outputMax = 1.0

    @Volatile
    private var integralMin = -1.0

    @Volatile
    private var integralMax = 1.0

    @Volatile
    private var deadband = 0.0

    // Getters for telemetry (thread-safe)
    // Controller state
    @get:Synchronized
    @set:Synchronized
    @Volatile
    var setpoint: Double = 0.0

    @Volatile
    private var lastError = 0.0

    @get:Synchronized
    @Volatile
    var integral: Double = 0.0
        private set

    @Volatile
    private var lastMeasurement = 0.0

    @Volatile
    private var lastSetpoint = 0.0

    @Volatile
    private var lastVelocity = 0.0

    // Timing
    private val timer = ElapsedTime()

    @Volatile
    private var lastTime = -1.0

    // Configuration flags
    @Volatile
    private var integralWindupProtection = true

    @Volatile
    private var derivativeKickPrevention = true

    @Volatile
    private var feedForwardEnabled = true

    // Telemetry data
    @get:Synchronized
    @Volatile
    var lastOutput: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var lastPIDoutput: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var lastProportional: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var lastIntegral: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var lastDerivative: Double = 0.0
        private set

    @get:Synchronized
    @Volatile
    var lastFeedForward: Double = 0.0
        private set

    /**
     * Constructor with basic PIDF gains
     */
    init {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        timer.reset()
    }

    /**
     * Constructor with full feed-forward control
     */
    constructor(
        kP: Double, kI: Double, kD: Double,
        kS: Double, kV: Double, kA: Double
    ) : this(kP, kI, kD, FeedForward(kS, kV, kA))

    /**
     * Calculate control output
     * @param measurement Current process variable value
     * @return Control output
     */
    @Synchronized
    fun calculate(measurement: Double): Double {
        return calculate(measurement, setpoint)
    }

    /**
     * Calculate control output with explicit setpoint
     * @param measurement Current process variable value
     * @param setpoint Desired setpoint
     * @return Control output
     */
    @Synchronized
    fun calculate(measurement: Double, setpoint: Double): Double {
        this.setpoint = setpoint

        val currentTime = timer.seconds()

        if (lastTime < 0.0) {
            lastTime = currentTime
            lastMeasurement = measurement
            lastSetpoint = setpoint
            lastError = setpoint - measurement

            val proportional = kP * lastError

            val ff = if (feedForwardEnabled) {
                feedForward.calculate(setpoint, 0.0)
            } else {
                0.0
            }

            val output = Range.clip(
                proportional + ff,
                outputMin,
                outputMax
            )

            lastOutput = output
            lastProportional = proportional
            lastIntegral = 0.0
            lastDerivative = 0.0
            lastFeedForward = ff

            return output
        }

        val dt = currentTime - lastTime

        // Calculate error
        val error = setpoint - measurement

        // Proportional term
        val proportional = kP * error

        // Integral term with windup protection
        if (dt > 0) {
            integral += error * dt

            if (integralWindupProtection) {
                integral = Range.clip(
                    integral, integralMin / (if (kI != 0.0) kI else 1.0),
                    integralMax / (if (kI != 0.0) kI else 1.0)
                )
            }
        }
        val integralTerm = kI * integral


        // Derivative term with kick prevention
        var derivative = 0.0
        if (dt > 0) {
            if (derivativeKickPrevention) {
                // Derivative on measurement to prevent kick
                derivative = -(measurement - lastMeasurement) / dt
            } else {
                // Traditional derivative on error
                derivative = (error - lastError) / dt
            }
        }
        val derivativeTerm = kD * derivative


        // Feed-forward term
        var ff = 0.0

        if (feedForwardEnabled) {
            val targetAcceleration =
                if (dt > 0.0)
                    (setpoint - lastSetpoint) / dt
                else
                    0.0
            ff = feedForward.calculate(setpoint, targetAcceleration)
        }


        // Calculate total output
        var output = proportional + integralTerm + derivativeTerm + ff

        // Apply deadband
        if (abs(output) < deadband) {
            output = 0.0
        }


        // Clamp output
        output = Range.clip(output, outputMin, outputMax)


        // Store telemetry data
        lastPIDoutput = proportional + integralTerm + derivativeTerm
        lastOutput = output
        lastProportional = proportional
        lastIntegral = integralTerm
        lastDerivative = derivativeTerm
        lastFeedForward = ff


        // Update state for next iteration
        lastError = error
        lastMeasurement = measurement
        lastSetpoint = setpoint
        lastTime = currentTime

        return output
    }

    /**
     * Reset controller state
     */
    @Synchronized
    fun reset() {
        lastError = 0.0
        integral = 0.0
        lastMeasurement = 0.0
        lastSetpoint = 0.0
        lastVelocity = 0.0
        lastTime = -1.0
        timer.reset()


        // Clear telemetry
        lastOutput = 0.0
        lastProportional = 0.0
        lastIntegral = 0.0
        lastDerivative = 0.0
        lastFeedForward = 0.0
    }

    // Setters for gains (thread-safe)
    @Synchronized
    fun setPIDF(kP: Double, kI: Double, kD: Double, feedForward: FeedForward) {
        this.kP = kP
        this.kI = kI
        this.kD = kD
        this.feedForward = feedForward
    }

    @Synchronized
    fun setFeedForward(kS: Double, kV: Double, kA: Double) {
        feedForward.kS = kS
        feedForward.kV = kV
        feedForward.kA = kA
    }

    @Synchronized
    fun setOutputLimits(min: Double, max: Double) {
        this.outputMin = min
        this.outputMax = max
    }

    @Synchronized
    fun setIntegralLimits(min: Double, max: Double) {
        this.integralMin = min
        this.integralMax = max
    }

    @Synchronized
    fun setDeadband(deadband: Double) {
        this.deadband = abs(deadband)
    }

    // Configuration methods
    @Synchronized
    fun setIntegralWindupProtection(enabled: Boolean) {
        this.integralWindupProtection = enabled
    }

    @Synchronized
    fun setDerivativeKickPrevention(enabled: Boolean) {
        this.derivativeKickPrevention = enabled
    }

    @Synchronized
    fun setFeedForwardEnabled(enabled: Boolean) {
        this.feedForwardEnabled = enabled
    }

    @Synchronized
    fun getLastError(): Double {
        return setpoint - lastMeasurement
    }

    @Synchronized
    fun getLastPOutput(): Double {
        return lastProportional
    }

    @Synchronized
    fun getLastIOutput(): Double {
        return lastIntegral
    }

    @Synchronized
    fun getLastDOutput(): Double {
        return lastDerivative
    }

    @Synchronized
    fun getLastPIDOutput(): Double {
        return lastPIDoutput
    }



    fun getFFoutput(): Double {
        return lastFeedForward
    }

    /**
     * Check if controller is at setpoint within tolerance
     */
    @Synchronized
    fun atSetpoint(tolerance: Double): Boolean {
        return abs(getLastError()) <= tolerance
    }

    @Synchronized
    override fun toString(): String =
        String.format(
            Locale.US,
            "SP:%.1f Err:%.1f Out:%.3f [P:%.3f I:%.3f D:%.3f FF:%.3f]",
            setpoint,
            getLastError(),
            lastOutput,
            lastProportional,
            lastIntegral,
            lastDerivative,
            lastFeedForward
        )
}