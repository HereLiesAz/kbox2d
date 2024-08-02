/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package de.pirckheimer_gymnasium.jbox2d.dynamics.joints;

import de.pirckheimer_gymnasium.jbox2d.common.Mat22;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Rot;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.SolverData;
import de.pirckheimer_gymnasium.jbox2d.pooling.IWorldPool;
//Point-to-point constraint
//Cdot = v2 - v1
//   = v2 + cross(w2, r2) - v1 - cross(w1, r1)
//J = [-I -r1_skew I r2_skew ]
//Identity used:
//w k % (rx i + ry j) = w * (-ry i + rx j)
//Angle constraint
//Cdot = w2 - w1
//J = [0 0 -1 0 0 1]
//K = invI1 + invI2

/**
 * A motor joint is used to control the relative motion between two bodies. A
 * typical usage is to control the movement of a dynamic body with respect to
 * the ground.
 *
 * @author Daniel Murphy
 */
public class MotorJoint extends Joint
{
    // Solver shared
    private final Vec2 linearOffset = new Vec2();

    private float angularOffset;

    private final Vec2 linearImpulse = new Vec2();

    private float angularImpulse;

    private float maxForce;

    private float maxTorque;

    private float correctionFactor;

    // Solver temp
    private int indexA;

    private int indexB;

    private final Vec2 rA = new Vec2();

    private final Vec2 rB = new Vec2();

    private final Vec2 localCenterA = new Vec2();

    private final Vec2 localCenterB = new Vec2();

    private final Vec2 linearError = new Vec2();

    private float angularError;

    private float invMassA;

    private float invMassB;

    private float invIA;

    private float invIB;

    private final Mat22 linearMass = new Mat22();

    private float angularMass;

    public MotorJoint(IWorldPool pool, MotorJointDef def)
    {
        super(pool, def);
        linearOffset.set(def.linearOffset);
        angularOffset = def.angularOffset;
        angularImpulse = 0.0f;
        maxForce = def.maxForce;
        maxTorque = def.maxTorque;
        correctionFactor = def.correctionFactor;
    }

    @Override
    public void getAnchorA(Vec2 out)
    {
        out.set(bodyA.getPosition());
    }

    @Override
    public void getAnchorB(Vec2 out)
    {
        out.set(bodyB.getPosition());
    }

    public void getReactionForce(float inv_dt, Vec2 out)
    {
        out.set(linearImpulse).mulLocal(inv_dt);
    }

    public float getReactionTorque(float inv_dt)
    {
        return angularImpulse * inv_dt;
    }

    public float getCorrectionFactor()
    {
        return correctionFactor;
    }

    public void setCorrectionFactor(float correctionFactor)
    {
        this.correctionFactor = correctionFactor;
    }

    /**
     * Set the target linear offset, in frame A, in meters.
     */
    public void setLinearOffset(Vec2 linearOffset)
    {
        if (linearOffset.x != this.linearOffset.x
                || linearOffset.y != this.linearOffset.y)
        {
            bodyA.setAwake(true);
            bodyB.setAwake(true);
            this.linearOffset.set(linearOffset);
        }
    }

    /**
     * Get the target linear offset, in frame A, in meters.
     */
    public void getLinearOffset(Vec2 out)
    {
        out.set(linearOffset);
    }

    /**
     * Get the target linear offset, in frame A, in meters. Do not modify.
     */
    public Vec2 getLinearOffset()
    {
        return linearOffset;
    }

    /**
     * Set the target angular offset, in radians.
     *
     * @param angularOffset
     */
    public void setAngularOffset(float angularOffset)
    {
        if (angularOffset != this.angularOffset)
        {
            bodyA.setAwake(true);
            bodyB.setAwake(true);
            this.angularOffset = angularOffset;
        }
    }

    public float getAngularOffset()
    {
        return angularOffset;
    }

    /**
     * Set the maximum friction force in N.
     *
     * @param force
     */
    public void setMaxForce(float force)
    {
        assert (force >= 0.0f);
        maxForce = force;
    }

    /**
     * Get the maximum friction force in N.
     */
    public float getMaxForce()
    {
        return maxForce;
    }

    /**
     * Set the maximum friction torque in N*m.
     */
    public void setMaxTorque(float torque)
    {
        assert (torque >= 0.0f);
        maxTorque = torque;
    }

    /**
     * Get the maximum friction torque in N*m.
     */
    public float getMaxTorque()
    {
        return maxTorque;
    }

    @Override
    public void initVelocityConstraints(SolverData data)
    {
        indexA = bodyA.islandIndex;
        indexB = bodyB.islandIndex;
        localCenterA.set(bodyA.sweep.localCenter);
        localCenterB.set(bodyB.sweep.localCenter);
        invMassA = bodyA.invMass;
        invMassB = bodyB.invMass;
        invIA = bodyA.invI;
        invIB = bodyB.invI;
        final Vec2 cA = data.positions[indexA].c;
        float aA = data.positions[indexA].a;
        final Vec2 vA = data.velocities[indexA].v;
        float wA = data.velocities[indexA].w;
        final Vec2 cB = data.positions[indexB].c;
        float aB = data.positions[indexB].a;
        final Vec2 vB = data.velocities[indexB].v;
        float wB = data.velocities[indexB].w;
        final Rot qA = pool.popRot();
        final Rot qB = pool.popRot();
        final Vec2 temp = pool.popVec2();
        Mat22 K = pool.popMat22();
        qA.set(aA);
        qB.set(aB);
        // Compute the effective mass matrix.
        // rA = b2Mul(qA, -localCenterA);
        // rB = b2Mul(qB, -localCenterB);
        rA.x = qA.c * -localCenterA.x - qA.s * -localCenterA.y;
        rA.y = qA.s * -localCenterA.x + qA.c * -localCenterA.y;
        rB.x = qB.c * -localCenterB.x - qB.s * -localCenterB.y;
        rB.y = qB.s * -localCenterB.x + qB.c * -localCenterB.y;
        // J = [-I -r1_skew I r2_skew]
        // [ 0 -1 0 1]
        // r_skew = [-ry; rx]
        // Matlab
        // K = [ mA+r1y^2*iA+mB+r2y^2*iB, -r1y*iA*r1x-r2y*iB*r2x,
        // -r1y*iA-r2y*iB]
        // [ -r1y*iA*r1x-r2y*iB*r2x, mA+r1x^2*iA+mB+r2x^2*iB, r1x*iA+r2x*iB]
        // [ -r1y*iA-r2y*iB, r1x*iA+r2x*iB, iA+iB]
        float mA = invMassA, mB = invMassB;
        float iA = invIA, iB = invIB;
        K.ex.x = mA + mB + iA * rA.y * rA.y + iB * rB.y * rB.y;
        K.ex.y = -iA * rA.x * rA.y - iB * rB.x * rB.y;
        K.ey.x = K.ex.y;
        K.ey.y = mA + mB + iA * rA.x * rA.x + iB * rB.x * rB.x;
        K.invertToOut(linearMass);
        angularMass = iA + iB;
        if (angularMass > 0.0f)
        {
            angularMass = 1.0f / angularMass;
        }
        // linearError = cB + rB - cA - rA - b2Mul(qA, linearOffset);
        Rot.mulToOutUnsafe(qA, linearOffset, temp);
        linearError.x = cB.x + rB.x - cA.x - rA.x - temp.x;
        linearError.y = cB.y + rB.y - cA.y - rA.y - temp.y;
        angularError = aB - aA - angularOffset;
        if (data.step.warmStarting)
        {
            // Scale impulses to support a variable time step.
            linearImpulse.x *= data.step.dtRatio;
            linearImpulse.y *= data.step.dtRatio;
            angularImpulse *= data.step.dtRatio;
            final Vec2 P = linearImpulse;
            vA.x -= mA * P.x;
            vA.y -= mA * P.y;
            wA -= iA * (rA.x * P.y - rA.y * P.x + angularImpulse);
            vB.x += mB * P.x;
            vB.y += mB * P.y;
            wB += iB * (rB.x * P.y - rB.y * P.x + angularImpulse);
        }
        else
        {
            linearImpulse.setZero();
            angularImpulse = 0.0f;
        }
        pool.pushVec2(1);
        pool.pushMat22(1);
        pool.pushRot(2);
        // data.velocities[indexA].v = vA;
        data.velocities[indexA].w = wA;
        // data.velocities[indexB].v = vB;
        data.velocities[indexB].w = wB;
    }

    @Override
    public void solveVelocityConstraints(SolverData data)
    {
        final Vec2 vA = data.velocities[indexA].v;
        float wA = data.velocities[indexA].w;
        final Vec2 vB = data.velocities[indexB].v;
        float wB = data.velocities[indexB].w;
        float mA = invMassA, mB = invMassB;
        float iA = invIA, iB = invIB;
        float h = data.step.dt;
        float inv_h = data.step.inv_dt;
        final Vec2 temp = pool.popVec2();
        // Solve angular friction
        {
            float Cdot = wB - wA + inv_h * correctionFactor * angularError;
            float impulse = -angularMass * Cdot;
            float oldImpulse = angularImpulse;
            float maxImpulse = h * maxTorque;
            angularImpulse = MathUtils.clamp(angularImpulse + impulse,
                    -maxImpulse, maxImpulse);
            impulse = angularImpulse - oldImpulse;
            wA -= iA * impulse;
            wB += iB * impulse;
        }
        final Vec2 Cdot = pool.popVec2();
        // Solve linear friction
        {
            // Cdot = vB + b2Cross(wB, rB) - vA - b2Cross(wA, rA) + inv_h *
            // correctionFactor *
            // linearError;
            Cdot.x = vB.x + -wB * rB.y - vA.x - -wA * rA.y
                    + inv_h * correctionFactor * linearError.x;
            Cdot.y = vB.y + wB * rB.x - vA.y - wA * rA.x
                    + inv_h * correctionFactor * linearError.y;
            final Vec2 impulse = temp;
            Mat22.mulToOutUnsafe(linearMass, Cdot, impulse);
            impulse.negateLocal();
            final Vec2 oldImpulse = pool.popVec2();
            oldImpulse.set(linearImpulse);
            linearImpulse.addLocal(impulse);
            float maxImpulse = h * maxForce;
            if (linearImpulse.lengthSquared() > maxImpulse * maxImpulse)
            {
                linearImpulse.normalize();
                linearImpulse.mulLocal(maxImpulse);
            }
            impulse.x = linearImpulse.x - oldImpulse.x;
            impulse.y = linearImpulse.y - oldImpulse.y;
            vA.x -= mA * impulse.x;
            vA.y -= mA * impulse.y;
            wA -= iA * (rA.x * impulse.y - rA.y * impulse.x);
            vB.x += mB * impulse.x;
            vB.y += mB * impulse.y;
            wB += iB * (rB.x * impulse.y - rB.y * impulse.x);
        }
        pool.pushVec2(3);
        // data.velocities[indexA].v.set(vA);
        data.velocities[indexA].w = wA;
        // data.velocities[indexB].v.set(vB);
        data.velocities[indexB].w = wB;
    }

    @Override
    public boolean solvePositionConstraints(SolverData data)
    {
        return true;
    }
}
