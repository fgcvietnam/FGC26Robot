package fgc.vietnam.robot01.Utils

import kotlin.math.sign

/**
 * Motor feedforward controller with static friction (kS), velocity (kV), and acceleration (kA) components.
 *
 * Formula: output = kS * sign(targetVelocity) + kV * targetVelocity + kA * targetAcceleration
 */
class FeedForward(
    var kS: Double = 0.0,
    var kV: Double = 0.0,
    var kA: Double = 0.0
) {
    /**
     * Calculates the feedforward output.
     *
     * @param targetVelocity The desired velocity.
     * @param targetAcceleration The desired acceleration.
     * @return The calculated feedforward output.
     */
    fun calculate(targetVelocity: Double, targetAcceleration: Double = 0.0): Double {
        val staticFriction = if (targetVelocity != 0.0) sign(targetVelocity) * kS else 0.0
        return staticFriction + (kV * targetVelocity) + (kA * targetAcceleration)
    }
}
