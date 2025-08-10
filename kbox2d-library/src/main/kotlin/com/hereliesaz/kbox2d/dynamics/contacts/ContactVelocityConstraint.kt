package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.common.Mat22
import com.hereliesaz.kbox2d.common.Settings
import com.hereliesaz.kbox2d.common.Vec2

class ContactVelocityConstraint {
    var points = Array(Settings.maxManifoldPoints) { VelocityConstraintPoint() }
    val normal = Vec2()
    val normalMass = Mat22()
    val K = Mat22()
    var indexA = 0
    var indexB = 0
    var invMassA = 0f
    var invMassB = 0f
    var invIA = 0f
    var invIB = 0f
    var friction = 0f
    var restitution = 0f
    var tangentSpeed = 0f
    var pointCount = 0
    var contactIndex = 0

    class VelocityConstraintPoint {
        val rA = Vec2()
        val rB = Vec2()
        var normalImpulse = 0f
        var tangentImpulse = 0f
        var normalMass = 0f
        var tangentMass = 0f
        var velocityBias = 0f
    }
}
