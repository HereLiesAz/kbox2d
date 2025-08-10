package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.collision.Manifold
import com.hereliesaz.kbox2d.collision.shapes.EdgeShape
import com.hereliesaz.kbox2d.collision.shapes.PolygonShape
import com.hereliesaz.kbox2d.collision.shapes.ShapeType
import com.hereliesaz.kbox2d.common.Transform
import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.pooling.IWorldPool

class EdgeAndPolygonContact(argPool: IWorldPool) : Contact(argPool) {

    init {
        m_typeA = ShapeType.EDGE
        m_typeB = ShapeType.POLYGON
    }

    override fun init(fA: Fixture, indexA: Int, fB: Fixture, indexB: Int) {
        super.init(fA, indexA, fB, indexB)
        assert(m_fixtureA!!.type == ShapeType.EDGE)
        assert(m_fixtureB!!.type == ShapeType.POLYGON)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        pool.collision.collideEdgeAndPolygon(
            manifold, m_fixtureA!!.shape as EdgeShape, xfA,
            m_fixtureB!!.shape as PolygonShape, xfB
        )
    }
}
