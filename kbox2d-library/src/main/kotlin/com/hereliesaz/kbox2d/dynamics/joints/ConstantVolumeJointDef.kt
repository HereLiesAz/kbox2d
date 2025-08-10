package com.hereliesaz.kbox2d.dynamics.joints

import com.hereliesaz.kbox2d.dynamics.Body
import java.util.ArrayList

class ConstantVolumeJointDef : JointDef(JointType.CONSTANT_VOLUME) {
    var frequencyHz = 0f
    var dampingRatio = 0f
    var bodies: ArrayList<Body> = ArrayList()
    var joints: ArrayList<DistanceJoint>? = null

    init {
        collideConnected = false
        frequencyHz = 0.0f
        dampingRatio = 0.0f
    }

    /**
     * Adds a body to the group
     *
     * @param argBody
     */
    fun addBody(argBody: Body) {
        bodies.add(argBody)
        if (bodies.size == 1) {
            bodyA = argBody
        }
        if (bodies.size == 2) {
            bodyB = argBody
        }
    }

    /**
     * Adds a body and the pre-made distance joint. Should only be used for deserialization.
     */
    fun addBodyAndJoint(argBody: Body, argJoint: DistanceJoint) {
        addBody(argBody)
        if (joints == null) {
            joints = ArrayList()
        }
        joints!!.add(argJoint)
    }
}
