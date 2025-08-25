package de.chaffic.joints

import de.chaffic.dynamics.Body
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2

/**
 * A joint that connects two bodies together.
 *
 * This joint acts like a spring-damper system, applying forces to both bodies to maintain
 * a specific distance between their attachment points.
 *
 * Example of creating a joint between two bodies:
 * ```kotlin
 * val bodyA = Body(Circle(20.0), 0.0, 200.0)
 * val bodyB = Body(Circle(20.0), 100.0, 200.0)
 *
 * val joint = JointToBody(
 *     bodyA, bodyB,
 *     naturalLength = 100.0,
 *     springConstant = 500.0,
 *     dampening = 10.0,
 *     canGoSlack = true,
 *     offset1 = Vec2(0.0, 0.0), // Attach to center of bodyA
 *     offset2 = Vec2(0.0, 0.0)  // Attach to center of bodyB
 * )
 *
 * world.addBody(bodyA)
 * world.addBody(bodyB)
 * world.addJoint(joint)
 * ```
 *
 * @property body2 The second body the joint is attached to.
 * @property offset2 The attachment point on the second body, in its local coordinates.
 * @property object2AttachmentPoint The attachment point on the second body, in world coordinates.
 *
 * @param body1 The first body to attach the joint to.
 * @param body2 The second body to attach the joint to.
 * @param jointLength The desired distance between the two bodies.
 * @param jointConstant The spring constant (stiffness) of the joint.
 * @param dampening The dampening factor to reduce oscillations.
 * @param canGoSlack Whether the joint can go slack (exert no force when compressed).
 * @param offset1 The offset from the first body's center to its attachment point.
 * @param offset2 The offset from the second body's center to its attachment point.
 */
class JointToBody(
    body1: Body,
    private val body2: Body,
    jointLength: Double,
    jointConstant: Double,
    dampening: Double,
    canGoSlack: Boolean,
    offset1: Vec2,
    private val offset2: Vec2
) : Joint(body1, jointLength, jointConstant, dampening, canGoSlack, offset1) {
    var object2AttachmentPoint: Vec2 = body2.position.plus(Mat2(body2.orientation).mul(offset2, Vec2()))

    /**
     * Applies the calculated tension force as impulses to the two connected bodies.
     */
    override fun applyTension() {
        val mat1 = Mat2(body.orientation)
        object1AttachmentPoint = body.position.plus(mat1.mul(offset, Vec2()))
        val mat2 = Mat2(body2.orientation)
        object2AttachmentPoint = body2.position.plus(mat2.mul(offset2, Vec2()))
        val tension = calculateTension()
        val distance = object2AttachmentPoint.minus(object1AttachmentPoint)
        distance.normalize()
        val impulse = distance.scalar(tension)
        body.applyLinearImpulse(impulse, object1AttachmentPoint.minus(body.position))
        body2.applyLinearImpulse(impulse.copyNegative(), object2AttachmentPoint.minus(body2.position))
    }

    /**
     * Calculates the tension force based on the current distance between the attachment points,
     * the spring constant, and the rate of change of extension.
     *
     * @return The calculated tension force.
     */
    override fun calculateTension(): Double {
        val distance = object1AttachmentPoint.minus(object2AttachmentPoint).length()
        if (distance < naturalLength && canGoSlack) {
            return .0
        }
        val extensionRatio = distance - naturalLength
        val tensionDueToHooksLaw = extensionRatio * springConstant
        val tensionDueToMotionDamping = dampeningConstant * rateOfChangeOfExtension()
        return tensionDueToHooksLaw + tensionDueToMotionDamping
    }

    /**
     * Calculates the rate of change of the distance between the two attachment points.
     * This is the relative velocity of the two points projected onto the vector connecting them.
     *
     * @return The rate of change of the joint's extension.
     */
    override fun rateOfChangeOfExtension(): Double {
        val distance = object2AttachmentPoint.minus(object1AttachmentPoint)
        distance.normalize()
        val relativeVelocity = body2.velocity.plus(
            object2AttachmentPoint.minus(body2.position).cross(body2.angularVelocity)
        ).minus(body.velocity).minus(
            object1AttachmentPoint.minus(body.position).cross(body.angularVelocity)
        )
        return relativeVelocity.dot(distance)
    }
}