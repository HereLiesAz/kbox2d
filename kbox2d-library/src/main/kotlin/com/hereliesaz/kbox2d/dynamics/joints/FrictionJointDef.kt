package com.hereliesaz.kbox2d.dynamics.joints

import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.dynamics.Body

/**
 * Friction joint definition.
 *
 * @author Daniel Murphy
 */
class FrictionJointDef : JointDef(JointType.FRICTION) {
    /**
     * The local anchor point relative to bodyA's origin.
     */
    val localAnchorA: Vec2

    /**
     * The local anchor point relative to bodyB's origin.
     */
    val localAnchorB: Vec2

    /**
     * The maximum friction force in N.
     */
    var maxForce: Float

    /**
     * The maximum friction torque in N-m.
     */
    var maxTorque: Float

    init {
        localAnchorA = Vec2()
        localAnchorB = Vec2()
        maxForce = 0f
        maxTorque = 0f
    }

    /**
     * Initialize the bodies, anchors, axis, and reference angle using the world anchor and world
     * axis.
     */
    fun initialize(bA: Body, bB: Body, anchor: Vec2) {
        bodyA = bA
        bodyB = bB
        bA.getLocalPointToOut(anchor, localAnchorA)
        bB.getLocalPointToOut(anchor, localAnchorB)
    }
}
