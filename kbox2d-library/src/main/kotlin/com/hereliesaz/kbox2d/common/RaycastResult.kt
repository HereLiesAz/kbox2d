package com.hereliesaz.kbox2d.common

// updated to rev 100
class RaycastResult {
    var lambda = 0.0f
    val normal = Vec2()
    fun set(argOther: RaycastResult): RaycastResult {
        lambda = argOther.lambda
        normal.set(argOther.normal)
        return this
    }
}
