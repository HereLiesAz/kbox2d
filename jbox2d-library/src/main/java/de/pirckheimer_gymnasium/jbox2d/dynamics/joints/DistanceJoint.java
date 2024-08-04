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
/*
 * JBox2D - A Java Port of Erin Catto's Box2D
 *
 * JBox2D homepage: http://jbox2d.sourceforge.net/
 * Box2D homepage: http://www.box2d.org
 *
 * This software is provided 'as-is', without any express or implied
 * warranty.  In no event will the authors be held liable for any damages
 * arising from the use of this software.
 *
 * Permission is granted to anyone to use this software for any purpose,
 * including commercial applications, and to alter it and redistribute it
 * freely, subject to the following restrictions:
 *
 * 1. The origin of this software must not be misrepresented; you must not
 * claim that you wrote the original software. If you use this software
 * in a product, an acknowledgment in the product documentation would be
 * appreciated but is not required.
 * 2. Altered source versions must be plainly marked as such, and must not be
 * misrepresented as being the original software.
 * 3. This notice may not be removed or altered from any source distribution.
 */
package de.pirckheimer_gymnasium.jbox2d.dynamics.joints;

import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Rot;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.SolverData;
import de.pirckheimer_gymnasium.jbox2d.pooling.IWorldPool;
//C = norm(p2 - p1) - L
//u = (p2 - p1) / norm(p2 - p1)
//Cdot = dot(u, v2 + cross(w2, r2) - v1 - cross(w1, r1))
//J = [-u -cross(r1, u) u cross(r2, u)]
//K = J * invM * JT
//= invMass1 + invI1 * cross(r1, u)^2 + invMass2 + invI2 * cross(r2, u)^2

/**
 * A distance joint constrains two points on two bodies to remain at a fixed
 * distance from each other. You can view this as a massless, rigid rod.
 *
 * @author Daniel Murphy
 */
public class DistanceJoint extends Joint
{
    private float frequencyHz;

    private float dampingRatio;

    private float bias;

    // Solver shared
    private final Vec2 localAnchorA;

    private final Vec2 localAnchorB;

    private float gamma;

    private float impulse;

    private float length;

    // Solver temp
    private int indexA;

    private int indexB;

    private final Vec2 u = new Vec2();

    private final Vec2 rA = new Vec2();

    private final Vec2 rB = new Vec2();

    private final Vec2 localCenterA = new Vec2();

    private final Vec2 localCenterB = new Vec2();

    private float invMassA;

    private float invMassB;

    private float invIA;

    private float invIB;

    private float mass;

    protected DistanceJoint(IWorldPool argWorld, final DistanceJointDef def)
    {
        super(argWorld, def);
        localAnchorA = def.localAnchorA.clone();
        localAnchorB = def.localAnchorB.clone();
        length = def.length;
        impulse = 0.0f;
        frequencyHz = def.frequencyHz;
        dampingRatio = def.dampingRatio;
        gamma = 0.0f;
        bias = 0.0f;
    }

    public void setFrequency(float hz)
    {
        frequencyHz = hz;
    }

    public float getFrequency()
    {
        return frequencyHz;
    }

    public float getLength()
    {
        return length;
    }

    public void setLength(float argLength)
    {
        length = argLength;
    }

    public void setDampingRatio(float damp)
    {
        dampingRatio = damp;
    }

    public float getDampingRatio()
    {
        return dampingRatio;
    }

    @Override
    public void getAnchorA(Vec2 argOut)
    {
        bodyA.getWorldPointToOut(localAnchorA, argOut);
    }

    @Override
    public void getAnchorB(Vec2 argOut)
    {
        bodyB.getWorldPointToOut(localAnchorB, argOut);
    }

    public Vec2 getLocalAnchorA()
    {
        return localAnchorA;
    }

    public Vec2 getLocalAnchorB()
    {
        return localAnchorB;
    }

    /**
     * Get the reaction force given the inverse time step. Unit is N.
     */
    @Override
    public void getReactionForce(float invDt, Vec2 argOut)
    {
        argOut.x = impulse * u.x * invDt;
        argOut.y = impulse * u.y * invDt;
    }

    /**
     * Get the reaction torque given the inverse time step. Unit is N*m. This is
     * always zero for a distance joint.
     */
    @Override
    public float getReactionTorque(float invDt)
    {
        return 0.0f;
    }

    @Override
    public void initVelocityConstraints(final SolverData data)
    {
        indexA = bodyA.islandIndex;
        indexB = bodyB.islandIndex;
        localCenterA.set(bodyA.sweep.localCenter);
        localCenterB.set(bodyB.sweep.localCenter);
        invMassA = bodyA.invMass;
        invMassB = bodyB.invMass;
        invIA = bodyA.invI;
        invIB = bodyB.invI;
        Vec2 cA = data.positions[indexA].c;
        float aA = data.positions[indexA].a;
        Vec2 vA = data.velocities[indexA].v;
        float wA = data.velocities[indexA].w;
        Vec2 cB = data.positions[indexB].c;
        float aB = data.positions[indexB].a;
        Vec2 vB = data.velocities[indexB].v;
        float wB = data.velocities[indexB].w;
        final Rot qA = pool.popRot();
        final Rot qB = pool.popRot();
        qA.set(aA);
        qB.set(aB);
        // use u as temporary variable
        Rot.mulToOutUnsafe(qA, u.set(localAnchorA).subLocal(localCenterA), rA);
        Rot.mulToOutUnsafe(qB, u.set(localAnchorB).subLocal(localCenterB), rB);
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);
        pool.pushRot(2);
        // Handle singularity.
        float length = u.length();
        if (length > Settings.linearSlop)
        {
            u.x *= 1.0f / length;
            u.y *= 1.0f / length;
        }
        else
        {
            u.set(0.0f, 0.0f);
        }
        float crAu = Vec2.cross(rA, u);
        float crBu = Vec2.cross(rB, u);
        float invMass = invMassA + invIA * crAu * crAu + invMassB
                + invIB * crBu * crBu;
        // Compute the effective mass matrix.
        mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
        if (frequencyHz > 0.0f)
        {
            float C = length - this.length;
            // Frequency
            float omega = 2.0f * MathUtils.PI * frequencyHz;
            // Damping coefficient
            float d = 2.0f * mass * dampingRatio * omega;
            // Spring stiffness
            float k = mass * omega * omega;
            // magic formulas
            float h = data.step.dt;
            gamma = h * (d + h * k);
            gamma = gamma != 0.0f ? 1.0f / gamma : 0.0f;
            bias = C * h * k * gamma;
            invMass += gamma;
            mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
        }
        else
        {
            gamma = 0.0f;
            bias = 0.0f;
        }
        if (data.step.warmStarting)
        {
            // Scale the impulse to support a variable time step.
            impulse *= data.step.dtRatio;
            Vec2 P = pool.popVec2();
            P.set(u).mulLocal(impulse);
            vA.x -= invMassA * P.x;
            vA.y -= invMassA * P.y;
            wA -= invIA * Vec2.cross(rA, P);
            vB.x += invMassB * P.x;
            vB.y += invMassB * P.y;
            wB += invIB * Vec2.cross(rB, P);
            pool.pushVec2(1);
        }
        else
        {
            impulse = 0.0f;
        }
//    data.velocities[indexA].v.set(vA);
        data.velocities[indexA].w = wA;
//    data.velocities[indexB].v.set(vB);
        data.velocities[indexB].w = wB;
    }

    @Override
    public void solveVelocityConstraints(final SolverData data)
    {
        Vec2 vA = data.velocities[indexA].v;
        float wA = data.velocities[indexA].w;
        Vec2 vB = data.velocities[indexB].v;
        float wB = data.velocities[indexB].w;
        final Vec2 vpA = pool.popVec2();
        final Vec2 vpB = pool.popVec2();
        // Cdot = dot(u, v + cross(w, r))
        Vec2.crossToOutUnsafe(wA, rA, vpA);
        vpA.addLocal(vA);
        Vec2.crossToOutUnsafe(wB, rB, vpB);
        vpB.addLocal(vB);
        float Cdot = Vec2.dot(u, vpB.subLocal(vpA));
        float impulse = -mass * (Cdot + bias + gamma * this.impulse);
        this.impulse += impulse;
        float Px = impulse * u.x;
        float Py = impulse * u.y;
        vA.x -= invMassA * Px;
        vA.y -= invMassA * Py;
        wA -= invIA * (rA.x * Py - rA.y * Px);
        vB.x += invMassB * Px;
        vB.y += invMassB * Py;
        wB += invIB * (rB.x * Py - rB.y * Px);
//    data.velocities[indexA].v.set(vA);
        data.velocities[indexA].w = wA;
//    data.velocities[indexB].v.set(vB);
        data.velocities[indexB].w = wB;
        pool.pushVec2(2);
    }

    @Override
    public boolean solvePositionConstraints(final SolverData data)
    {
        if (frequencyHz > 0.0f)
        {
            return true;
        }
        final Rot qA = pool.popRot();
        final Rot qB = pool.popRot();
        final Vec2 rA = pool.popVec2();
        final Vec2 rB = pool.popVec2();
        final Vec2 u = pool.popVec2();
        Vec2 cA = data.positions[indexA].c;
        float aA = data.positions[indexA].a;
        Vec2 cB = data.positions[indexB].c;
        float aB = data.positions[indexB].a;
        qA.set(aA);
        qB.set(aB);
        Rot.mulToOutUnsafe(qA, u.set(localAnchorA).subLocal(localCenterA), rA);
        Rot.mulToOutUnsafe(qB, u.set(localAnchorB).subLocal(localCenterB), rB);
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);
        float length = u.normalize();
        float C = length - this.length;
        C = MathUtils.clamp(C, -Settings.maxLinearCorrection,
                Settings.maxLinearCorrection);
        float impulse = -mass * C;
        float Px = impulse * u.x;
        float Py = impulse * u.y;
        cA.x -= invMassA * Px;
        cA.y -= invMassA * Py;
        aA -= invIA * (rA.x * Py - rA.y * Px);
        cB.x += invMassB * Px;
        cB.y += invMassB * Py;
        aB += invIB * (rB.x * Py - rB.y * Px);
//    data.positions[indexA].c.set(cA);
        data.positions[indexA].a = aA;
//    data.positions[indexB].c.set(cB);
        data.positions[indexB].a = aB;
        pool.pushVec2(3);
        pool.pushRot(2);
        return MathUtils.abs(C) < Settings.linearSlop;
    }
}
