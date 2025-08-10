package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.collision.Manifold
import com.hereliesaz.kbox2d.collision.shapes.CircleShape
import com.hereliesaz.kbox2d.collision.shapes.ShapeType
import com.hereliesaz.kbox2d.common.Transform
import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.pooling.IWorldPool

class CircleContact(argPool: IWorldPool) : Contact(argPool) {

    init {
        m_typeA = ShapeType.CIRCLE
        m_typeB = ShapeType.CIRCLE
    }

    fun init(fixtureA: Fixture, fixtureB: Fixture) {
        super.init(fixtureA, 0, fixtureB, 0)
        assert(m_fixtureA!!.type == ShapeType.CIRCLE)
        assert(m_fixtureB!!.type == ShapeType.CIRCLE)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        pool.collision.collideCircles(
            manifold, m_fixtureA!!.shape as CircleShape, xfA,
            m_fixtureB!!.shape as CircleShape, xfB
        )
    }
}
