package com.hereliesaz.kbox2d.dynamics.joints

import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.dynamics.Body

//Updated to rev 56->130->142 of b2DistanceJoint.cpp/.h
/**
 * Distance joint definition. This requires defining an anchor point on both bodies and the non-zero
 * length of the distance joint. The definition uses local anchor points so that the initial
 * configuration can violate the constraint slightly. This helps when saving and loading a game.
 *
 * @warning Do not use a zero or short length.
 */
class DistanceJointDef : JointDef(JointType.DISTANCE) {
    /** The local anchor point relative to body1's origin.  */
    val localAnchorA: Vec2

    /** The local anchor point relative to body2's origin.  */
    val localAnchorB: Vec2

    /** The equilibrium length between the anchor points.  */
    var length: Float

    /**
     * The mass-spring-damper frequency in Hertz.
     */
    var frequencyHz: Float

    /**
     * The damping ratio. 0 = no damping, 1 = critical damping.
     */
    var dampingRatio: Float

    init {
        localAnchorA = Vec2(0.0f, 0.0f)
        localAnchorB = Vec2(0.0f, 0.0f)
        length = 1.0f
        frequencyHz = 0.0f
        dampingRatio = 0.0f
    }

    /**
     * Initialize the bodies, anchors, and length using the world anchors.
     *
     * @param b1 First body
     * @param b2 Second body
     * @param anchor1 World anchor on first body
     * @param anchor2 World anchor on second body
     */
    fun initialize(b1: Body, b2: Body, anchor1: Vec2, anchor2: Vec2) {
        bodyA = b1
        bodyB = b2
        localAnchorA.set(bodyA.getLocalPoint(anchor1))
        localAnchorB.set(bodyB.getLocalPoint(anchor2))
        val d = anchor2.sub(anchor1)
        length = d.length()
    }
}
