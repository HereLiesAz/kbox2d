package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.collision.Manifold.ManifoldType
import com.hereliesaz.kbox2d.common.Settings
import com.hereliesaz.kbox2d.common.Vec2

class ContactPositionConstraint {
    var localPoints = Array(Settings.maxManifoldPoints) { Vec2() }
    val localNormal = Vec2()
    val localPoint = Vec2()
    var indexA = 0
    var indexB = 0
    var invMassA = 0f
    var invMassB = 0f
    val localCenterA = Vec2()
    val localCenterB = Vec2()
    var invIA = 0f
    var invIB = 0f
    lateinit var type: ManifoldType
    var radiusA = 0f
    var radiusB = 0f
    var pointCount = 0
}
