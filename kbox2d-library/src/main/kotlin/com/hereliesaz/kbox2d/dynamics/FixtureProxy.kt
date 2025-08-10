package com.hereliesaz.kbox2d.dynamics

import com.hereliesaz.kbox2d.collision.AABB

/**
 * This proxy is used internally to connect fixtures to the broad-phase.
 *
 * @author Daniel
 */
class FixtureProxy {
    val aabb = AABB()
    lateinit var fixture: Fixture
    var childIndex = 0
    var proxyId = 0
}
