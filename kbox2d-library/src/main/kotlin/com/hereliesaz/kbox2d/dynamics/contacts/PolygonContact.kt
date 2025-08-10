package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.collision.Manifold
import com.hereliesaz.kbox2d.collision.shapes.PolygonShape
import com.hereliesaz.kbox2d.collision.shapes.ShapeType
import com.hereliesaz.kbox2d.common.Transform
import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.pooling.IWorldPool

class PolygonContact(argPool: IWorldPool) : Contact(argPool) {

    init {
        m_typeA = ShapeType.POLYGON
        m_typeB = ShapeType.POLYGON
    }

    fun init(fixtureA: Fixture, fixtureB: Fixture) {
        super.init(fixtureA, 0, fixtureB, 0)
        assert(m_fixtureA!!.type == ShapeType.POLYGON)
        assert(m_fixtureB!!.type == ShapeType.POLYGON)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        pool.collision.collidePolygons(
            manifold, m_fixtureA!!.shape as PolygonShape, xfA,
            m_fixtureB!!.shape as PolygonShape, xfB
        )
    }
}
