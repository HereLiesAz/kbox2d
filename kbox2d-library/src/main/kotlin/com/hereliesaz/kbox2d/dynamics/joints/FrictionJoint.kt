package com.hereliesaz.kbox2d.dynamics.joints

import com.hereliesaz.kbox2d.common.Mat22
import com.hereliesaz.kbox2d.common.MathUtils
import com.hereliesaz.kbox2d.common.Rot
import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.dynamics.SolverData
import com.hereliesaz.kbox2d.pooling.IWorldPool

class FrictionJoint(argWorldPool: IWorldPool, def: FrictionJointDef) : Joint(argWorldPool, def) {
    val localAnchorA: Vec2
    val localAnchorB: Vec2

    // Solver shared
    private val m_linearImpulse: Vec2
    private var m_angularImpulse: Float
    var maxForce: Float
    var maxTorque: Float

    // Solver temp
    private var m_indexA = 0
    private var m_indexB = 0
    private val m_rA = Vec2()
    private val m_rB = Vec2()
    private val m_localCenterA = Vec2()
    private val m_localCenterB = Vec2()
    private var m_invMassA = 0f
    private var m_invMassB = 0f
    private var m_invIA = 0f
    private var m_invIB = 0f
    private val m_linearMass = Mat22()
    private var m_angularMass = 0f

    override fun getAnchorA(argOut: Vec2) {
        m_bodyA.getWorldPointToOut(localAnchorA, argOut)
    }

    override fun getAnchorB(argOut: Vec2) {
        m_bodyB.getWorldPointToOut(localAnchorB, argOut)
    }

    override fun getReactionForce(inv_dt: Float, argOut: Vec2) {
        argOut.set(m_linearImpulse).mulLocal(inv_dt)
    }

    override fun getReactionTorque(inv_dt: Float): Float {
        return inv_dt * m_angularImpulse
    }

    /**
     * @see org.jbox2d.dynamics.joints.Joint.initVelocityConstraints
     */
    override fun initVelocityConstraints(data: SolverData) {
        m_indexA = m_bodyA.m_islandIndex
        m_indexB = m_bodyB.m_islandIndex
        m_localCenterA.set(m_bodyA.m_sweep.localCenter)
        m_localCenterB.set(m_bodyB.m_sweep.localCenter)
        m_invMassA = m_bodyA.m_invMass
        m_invMassB = m_bodyB.m_invMass
        m_invIA = m_bodyA.m_invI
        m_invIB = m_bodyB.m_invI
        val aA = data.positions[m_indexA].a
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val aB = data.positions[m_indexB].a
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val temp = pool.popVec2()
        val qA = pool.popRot()
        val qB = pool.popRot()
        qA.set(aA)
        qB.set(aB)

        // Compute the effective mass matrix.
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(m_localCenterA), m_rA)
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(m_localCenterB), m_rB)

        // J = [-I -r1_skew I r2_skew]
        // [ 0 -1 0 1]
        // r_skew = [-ry; rx]

        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x, -r1y*iA-r2y*iB]
        // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
        // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val K = pool.popMat22()
        K.ex.x = mA + mB + iA * m_rA.y * m_rA.y + iB * m_rB.y * m_rB.y
        K.ex.y = -iA * m_rA.x * m_rA.y - iB * m_rB.x * m_rB.y
        K.ey.x = K.ex.y
        K.ey.y = mA + mB + iA * m_rA.x * m_rA.x + iB * m_rB.x * m_rB.x
        K.invertToOut(m_linearMass)
        m_angularMass = iA + iB
        if (m_angularMass > 0.0f) {
            m_angularMass = 1.0f / m_angularMass
        }
        if (data.step.warmStarting) {
            // Scale impulses to support a variable time step.
            m_linearImpulse.mulLocal(data.step.dtRatio)
            m_angularImpulse *= data.step.dtRatio
            val P = pool.popVec2()
            P.set(m_linearImpulse)
            temp.set(P).mulLocal(mA)
            vA.subLocal(temp)
            wA -= iA * (Vec2.cross(m_rA, P) + m_angularImpulse)
            temp.set(P).mulLocal(mB)
            vB.addLocal(temp)
            wB += iB * (Vec2.cross(m_rB, P) + m_angularImpulse)
            pool.pushVec2(1)
        } else {
            m_linearImpulse.setZero()
            m_angularImpulse = 0.0f
        }
        //    data.velocities[m_indexA].v.set(vA);
        if (data.velocities[m_indexA].w != wA) {
            assert(data.velocities[m_indexA].w != wA)
        }
        data.velocities[m_indexA].w = wA
        //    data.velocities[m_indexB].v.set(vB);
        data.velocities[m_indexB].w = wB
        pool.pushRot(2)
        pool.pushVec2(1)
        pool.pushMat22(1)
    }

    override fun solveVelocityConstraints(data: SolverData) {
        val vA = data.velocities[m_indexA].v
        var wA = data.velocities[m_indexA].w
        val vB = data.velocities[m_indexB].v
        var wB = data.velocities[m_indexB].w
        val mA = m_invMassA
        val mB = m_invMassB
        val iA = m_invIA
        val iB = m_invIB
        val h = data.step.dt

        // Solve angular friction
        run {
            val Cdot = wB - wA
            var impulse = -m_angularMass * Cdot
            val oldImpulse = m_angularImpulse
            val maxImpulse = h * maxTorque
            m_angularImpulse = MathUtils.clamp(m_angularImpulse + impulse, -maxImpulse, maxImpulse)
            impulse = m_angularImpulse - oldImpulse
            wA -= iA * impulse
            wB += iB * impulse
        }

        // Solve linear friction
        run {
            val Cdot = pool.popVec2()
            val temp = pool.popVec2()
            Vec2.crossToOutUnsafe(wA, m_rA, temp)
            Vec2.crossToOutUnsafe(wB, m_rB, Cdot)
            Cdot.addLocal(vB).subLocal(vA).subLocal(temp)
            val impulse = pool.popVec2()
            Mat22.mulToOutUnsafe(m_linearMass, Cdot, impulse)
            impulse.negateLocal()
            val oldImpulse = pool.popVec2()
            oldImpulse.set(m_linearImpulse)
            m_linearImpulse.addLocal(impulse)
            val maxImpulse = h * maxForce
            if (m_linearImpulse.lengthSquared() > maxImpulse * maxImpulse) {
                m_linearImpulse.normalize()
                m_linearImpulse.mulLocal(maxImpulse)
            }
            impulse.set(m_linearImpulse).subLocal(oldImpulse)
            temp.set(impulse).mulLocal(mA)
            vA.subLocal(temp)
            wA -= iA * Vec2.cross(m_rA, impulse)
            temp.set(impulse).mulLocal(mB)
            vB.addLocal(temp)
            wB += iB * Vec2.cross(m_rB, impulse)
        }

//    data.velocities[m_indexA].v.set(vA);
        if (data.velocities[m_indexA].w != wA) {
            assert(data.velocities[m_indexA].w != wA)
        }
        data.velocities[m_indexA].w = wA

//    data.velocities[m_indexB].v.set(vB);
        data.velocities[m_indexB].w = wB
        pool.pushVec2(4)
    }

    override fun solvePositionConstraints(data: SolverData): Boolean {
        return true
    }

    init {
        localAnchorA = Vec2(def.localAnchorA)
        localAnchorB = Vec2(def.localAnchorB)
        m_linearImpulse = Vec2()
        m_angularImpulse = 0.0f
        maxForce = def.maxForce
        maxTorque = def.maxTorque
    }
}
