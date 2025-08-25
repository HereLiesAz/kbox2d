package de.chaffic.joints

import de.chaffic.dynamics.Body
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2

/**
 * A joint that connects a body to a fixed point in world space.
 *
 * This is useful for creating things like pendulums or tethers. The joint applies forces
 * to the body to keep it at a certain distance from the anchor point.
 *
 * Example of creating a pendulum:
 * ```kotlin
 * val anchorPoint = Vec2(200.0, 300.0)
 * val pendulumBob = Body(Circle(20.0), 200.0, 100.0)
 *
 * val joint = JointToPoint(
 *     pendulumBob,
 *     anchorPoint,
 *     naturalLength = 200.0,
 *     springConstant = 1000.0,
 *     dampening = 20.0,
 *     canGoSlack = false,
 *     offset = Vec2(0.0, 0.0) // Attach to the center of the bob
 * )
 *
 * world.addBody(pendulumBob)
 * world.addJoint(joint)
 * ```
 *
 * @property pointAttachedTo The fixed point in world space that the joint is attached to.
 *
 * @param b1 The body to attach the joint to.
 * @param pointAttachedTo The fixed anchor point for the joint in world coordinates.
 * @param jointLength The desired distance between the body and the anchor point.
 * @param jointConstant The spring constant (stiffness) of the joint.
 * @param dampening The dampening factor to reduce oscillations.
 * @param canGoSlack Whether the joint can go slack (exert no force when compressed).
 * @param offset The offset from the body's center to its attachment point.
 */
class JointToPoint(
    b1: Body,
    val pointAttachedTo: Vec2,
    jointLength: Double,
    jointConstant: Double,
    dampening: Double,
    canGoSlack: Boolean,
    offset: Vec2
) : Joint(b1, jointLength, jointConstant, dampening, canGoSlack, offset) {

    /**
     * Applies the calculated tension force as an impulse to the connected body.
     */
    override fun applyTension() {
        val mat1 = Mat2(body.orientation)
        object1AttachmentPoint = body.position.plus(mat1.mul(offset, Vec2()))
        val tension = calculateTension()
        val distance = pointAttachedTo.minus(object1AttachmentPoint)
        distance.normalize()
        val impulse = distance.scalar(tension)
        body.applyLinearImpulse(impulse, object1AttachmentPoint.minus(body.position))
    }

    /**
     * Calculates the tension force based on the current distance between the body's attachment point
     * and the fixed world point.
     *
     * @return The calculated tension force.
     */
    override fun calculateTension(): Double {
        val distance = object1AttachmentPoint.minus(pointAttachedTo).length()
        if (distance < naturalLength && canGoSlack) {
            return .0
        }
        val extensionRatio = distance - naturalLength
        val tensionDueToHooksLaw = extensionRatio * springConstant
        val tensionDueToMotionDamping = dampeningConstant * rateOfChangeOfExtension()
        return tensionDueToHooksLaw + tensionDueToMotionDamping
    }

    /**
     * Calculates the rate of change of the distance between the body's attachment point and the fixed world point.
     *
     * @return The rate of change of the joint's extension.
     */
    override fun rateOfChangeOfExtension(): Double {
        val distance = pointAttachedTo.minus(object1AttachmentPoint)
        distance.normalize()
        val relativeVelocity = body.velocity.copyNegative()
            .minus(object1AttachmentPoint.minus(body.position).cross(body.angularVelocity))
        return relativeVelocity.dot(distance)
    }
}