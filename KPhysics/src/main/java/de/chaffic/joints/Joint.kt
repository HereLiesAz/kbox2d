package de.chaffic.joints

import de.chaffic.dynamics.Body
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2

/**
 * The base class for all joints in the physics engine. A joint is a constraint that connects
 * one or more bodies. This class provides the common properties and methods for all joint types.
 *
 * This class is abstract and cannot be instantiated directly. Instead, use one of the concrete
 * joint implementations:
 * - [JointToBody]: A joint that connects two bodies.
 * - [JointToPoint]: A joint that connects a body to a fixed point in the world.
 *
 * @property body The first body the joint is attached to.
 * @property naturalLength The desired resting distance of the joint. The joint will apply forces
 * to try and maintain this distance.
 * @property springConstant The spring constant (stiffness) of the joint. Higher values result in a stiffer joint.
 * @property dampeningConstant The dampening constant, which helps to reduce oscillations.
 * @property canGoSlack If `true`, the joint will not apply any force when its length is less than the `naturalLength`.
 * @property offset The attachment point of the joint on the first body, in the body's local coordinates.
 * @property object1AttachmentPoint The attachment point of the joint on the first body, in world coordinates.
 *
 * @param body The first body to attach the joint to.
 * @param naturalLength The desired distance between the joint's attachment points.
 * @param springConstant The strength (stiffness) of the joint.
 * @param dampeningConstant The dampening factor to reduce oscillations.
 * @param canGoSlack Whether the joint can go slack (exert no force when compressed).
 * @param offset The offset from the body's center to the attachment point, in local coordinates.
 */
abstract class Joint protected constructor(
    protected val body: Body,
    protected val naturalLength: Double,
    protected val springConstant: Double,
    protected val dampeningConstant: Double,
    protected val canGoSlack: Boolean,
    protected val offset: Vec2
) {
    var object1AttachmentPoint: Vec2

    init {
        val u = Mat2(body.orientation)
        object1AttachmentPoint = body.position.plus(u.mul(offset, Vec2()))
    }

    /**
     * Applies the tension forces of the joint to the attached bodies.
     * This method is called by the world during each simulation step.
     */
    abstract fun applyTension()

    /**
     * Calculates the current tension force of the joint based on its extension and velocity.
     *
     * @return The calculated tension force. A positive value indicates tension (pulling),
     * while a negative value indicates compression (pushing).
     */
    abstract fun calculateTension(): Double

    /**
     * Calculates the rate of change of the joint's length.
     * This is used to apply dampening forces.
     *
     * @return The rate of change of the joint's extension.
     */
    abstract fun rateOfChangeOfExtension(): Double
}