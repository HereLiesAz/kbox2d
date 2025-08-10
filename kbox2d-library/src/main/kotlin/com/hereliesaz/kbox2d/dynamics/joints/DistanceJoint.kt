package com.hereliesaz.kbox2d.dynamics.joints

import com.hereliesaz.kbox2d.common.MathUtils
import com.hereliesaz.kbox2d.common.Rot
import com.hereliesaz.kbox2d.common.Settings
import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.dynamics.SolverData
import com.hereliesaz.kbox2d.pooling.IWorldPool

//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2
/**
 * A distance joint constrains two points on two bodies to remain at a fixed distance from each
 * other. You can view this as a massless, rigid rod.
 */
class DistanceJoint(argWorld: IWorldPool, def: DistanceJointDef) : Joint(argWorld, def) {
    var frequencyHz: Float
    var dampingRatio: Float
    private var m_bias: Float

    // Solver shared
    val localAnchorA: Vec2
    val localAnchorB: Vec2
    private var m_gamma: Float
    private var m_impulse: Float
    var length: Float

    // Solver temp
    private var m_indexA = 0
    private var m_indexB = 0
    private val m_u = Vec2()
    private val m_rA = Vec2()
    private val m_rB = Vec2()
    private val m_localCenterA = Vec2()
    private val m_localCenterB = Vec2()
    private var m_invMassA = 0f
    private var m_invMassB = 0f
    private var m_invIA = 0f
    private var m_invIB = 0f
    private var m_mass = 0f
    fun setFrequency(hz: Float) {
        frequencyHz = hz
    }

    override fun getAnchorA(argOut: Vec2) {
        m_bodyA.getWorldPointToOut(localAnchorA, argOut)
    }

    override fun getAnchorB(argOut: Vec2) {
        m_bodyB.getWorldPointToOut(localAnchorB, argOut)
    }

    /**
     * Get the reaction force given the inverse time step. Unit is N.
     */
    override fun getReactionForce(inv_dt: Float, argOut: Vec2) {
        argOut.x = m_impulse * m_u.x * inv_dt
        argOut.y = m_impulse * m_u.y * inv_dt
    }

    /**
     * Get the reaction torque given the inverse time step. Unit is N*m. This is always zero for a
     * distance joint.
     */
    override fun getReactionTorque(inv_dt: Float): Float {
        return 0.0f
    }

    override fun initVelocityConstraints(data: SolverData) {
        m_indexA = m_bodyA.m_islandIndex
        m_indexB = m_bodyB.m_islandIndex
        m_localCenterA.set(m_bodyA.m_sweep.localCenter)
        m_localCenterB.set(m_bodyB.m_sweep.localCenter)
        m_invMassA = m_bodyA.m_invMass
        m_invMassB = m_bodyB.m_invMass
        m_invIA = m_bodyA.m_invI
        m_invIB = m_bodyB.m_invI
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val qA = pool.popRot()
        val qB = pool.popRot()
        qA.set(aA)
        qB.set(aB)

        // use m_u as temporary variable
        Rot.mulToOutUnsafe(qA, m_u.set(localAnchorA).subLocal(m_localCenterA), m_rA)
        Rot.mulToOutUnsafe(qB, m_u.set(localAnchorB).subLocal(m_localCenterB), m_rB)
        m_u.set(cB).addLocal(m_rB).subLocal(cA).subLocal(m_rA)
        pool.pushRot(2)

        // Handle singularity.
        val length = m_u.length()
        if (length > Settings.linearSlop) {
            m_u.x *= 1.0f / length
            m_u.y *= 1.0f / length
        } else {
            m_u.set(0.0f, 0.0f)
        }
        val crAu = Vec2.cross(m_rA, m_u)
        val crBu = Vec2.cross(m_rB, m_u)
        var invMass = m_invMassA + m_invIA * crAu * crAu + m_invMassB + m_invIB * crBu * crBu

        // Compute the effective mass matrix.
        m_mass = if (invMass != 0.0f) 1.0f / invMass else 0.0f
        if (frequencyHz > 0.0f) {
            val C = length - this.length

            // Frequency
            val omega = 2.0f * MathUtils.PI * frequencyHz

            // Damping coefficient
            val d = 2.0f * m_mass * dampingRatio * omega

            // Spring stiffness
            val k = m_mass * omega * omega

            // magic formulas
            val h = data.step.dt
            m_gamma = h * (d + h * k)
            m_gamma = if (m_gamma != 0.0f) 1.0f / m_gamma else 0.0f
            m_bias = C * h * k * m_gamma
            invMass += m_gamma
            m_mass = if (invMass != 0.0f) 1.0f / invMass else 0.0f
        } else {
            m_gamma = 0.0f
            m_bias = 0.0f
        }
        if (data.step.warmStarting) {

            // Scale the impulse to support a variable time step.
            m_impulse *= data.step.dtRatio
            val P = pool.popVec2()
            P.set(m_u).mulLocal(m_impulse)
            vA.x -= m_invMassA * P.x
            vA.y -= m_invMassA * P.y
            wA -= m_invIA * Vec2.cross(m_rA, P)
            vB.x += m_invMassB * P.x
            vB.y += m_invMassB * P.y
            wB += m_invIB * Vec2.cross(m_rB, P)
            pool.pushVec2(1)
        } else {
            m_impulse = 0.0f
        }
        //    data.velocities[m_indexA].v.set(vA);
        data.velocities[m_indexA].w = wA
        //    data.velocities[m_indexB].v.set(vB);
        data.velocities[m_indexB].w = wB
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val vpA = pool.popVec2()
        val vpB = pool.popVec2()

        // Cdot = dot(u, v + cross(w, r))
        Vec2.crossToOutUnsafe(wA, m_rA, vpA)
        vpA.addLocal(vA)
        Vec2.crossToOutUnsafe(wB, m_rB, vpB)
        vpB.addLocal(vB)
        val Cdot = Vec2.dot(m_u, vpB.subLocal(vpA))
        val impulse = -m_mass * (Cdot + m_bias + m_gamma * m_impulse)
        m_impulse += impulse
        val Px = impulse * m_u.x
        val Py = impulse * m_u.y
        vA.x -= m_invMassA * Px
        vA.y -= m_invMassA * Py
        wA -= m_invIA * (m_rA.x * Py - m_rA.y * Px)
        vB.x += m_invMassB * Px
        vB.y += m_invMassB * Py
        wB += m_invIB * (m_rB.x * Py - m_rB.y * Px)

//    data.velocities[m_indexA].v.set(vA);
        data.velocities[m_indexA].w = wA
        //    data.velocities[m_indexB].v.set(vB);
        data.velocities[m_indexB].w = wB
        pool.pushVec2(2)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        if (frequencyHz > 0.0f) {
            return true
        }
        val qA = pool.popRot()
        val qB = pool.popRot()
        val rA = pool.popVec2()
        val rB = pool.popVec2()
        val u = pool.popVec2()
        val cA = data.positions[m_indexA].c
        var aA = data.positions[m_indexA].a
        val cB = data.positions[m_indexB].c
        var aB = data.positions[m_indexB].a
        qA.set(aA)
        qB.set(aB)
        Rot.mulToOutUnsafe(qA, u.set(localAnchorA).subLocal(m_localCenterA), rA)
        Rot.mulToOutUnsafe(qB, u.set(localAnchorB).subLocal(m_localCenterB), rB)
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA)
        val length = u.normalize()
        var C = length - this.length
        C = MathUtils.clamp(C, -Settings.maxLinearCorrection, Settings.maxLinearCorrection)
        val impulse = -m_mass * C
        val Px = impulse * u.x
        val Py = impulse * u.y
        cA.x -= m_invMassA * Px
        cA.y -= m_invMassA * Py
        aA -= m_invIA * (rA.x * Py - rA.y * Px)
        cB.x += m_invMassB * Px
        cB.y += m_invMassB * Py
        aB += m_invIB * (rB.x * Py - rB.y * Px)

//    data.positions[m_indexA].c.set(cA);
        data.positions[m_indexA].a = aA
        //    data.positions[m_indexB].c.set(cB);
        data.positions[m_indexB].a = aB
        pool.pushVec2(3)
        pool.pushRot(2)
        return MathUtils.abs(C) < Settings.linearSlop
    }

    init {
        localAnchorA = def.localAnchorA.clone()
        localAnchorB = def.localAnchorB.clone()
        length = def.length
        m_impulse = 0.0f
        frequencyHz = def.frequencyHz
        dampingRatio = def.dampingRatio
        m_gamma = 0.0f
        m_bias = 0.0f
    }
}
