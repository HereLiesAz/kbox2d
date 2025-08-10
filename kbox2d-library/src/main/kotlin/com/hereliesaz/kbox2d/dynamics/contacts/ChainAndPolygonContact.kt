package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.collision.Manifold
import com.hereliesaz.kbox2d.collision.shapes.ChainShape
import com.hereliesaz.kbox2d.collision.shapes.EdgeShape
import com.hereliesaz.kbox2d.collision.shapes.PolygonShape
import com.hereliesaz.kbox2d.collision.shapes.ShapeType
import com.hereliesaz.kbox2d.common.Transform
import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.pooling.IWorldPool

class ChainAndPolygonContact(argPool: IWorldPool) : Contact(argPool) {
    private val edge = EdgeShape()

    init {
        m_typeA = ShapeType.CHAIN
        m_typeB = ShapeType.POLYGON
    }

    override fun init(fA: Fixture, indexA: Int, fB: Fixture, indexB: Int) {
        super.init(fA, indexA, fB, indexB)
        assert(m_fixtureA!!.type == ShapeType.CHAIN)
        assert(m_fixtureB!!.type == ShapeType.POLYGON)
    }

    override fun evaluate(manifold: Manifold, xfA: Transform, xfB: Transform) {
        val chain = m_fixtureA!!.shape as ChainShape
        chain.getChildEdge(edge, m_indexA)
        pool.collision.collideEdgeAndPolygon(
            manifold, edge, xfA,
            m_fixtureB!!.shape as PolygonShape, xfB
        )
    }
}
