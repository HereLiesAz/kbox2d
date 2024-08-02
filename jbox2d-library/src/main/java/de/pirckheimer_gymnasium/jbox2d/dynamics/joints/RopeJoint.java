package de.pirckheimer_gymnasium.jbox2d.dynamics.joints;

import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Rot;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.SolverData;
import de.pirckheimer_gymnasium.jbox2d.pooling.IWorldPool;

/**
 * A rope joint enforces a maximum distance between two points on two bodies. It
 * has no other effect. Warning: if you attempt to change the maximum length
 * during the simulation you will get some non-physical behavior. A model that
 * would allow you to dynamically modify the length would have some sponginess,
 * so I chose not to implement it that way. See DistanceJoint if you want to
 * dynamically control length.
 *
 * @author Daniel Murphy
 */
public class RopeJoint extends Joint
{
    // Solver shared
    private final Vec2 localAnchorA = new Vec2();

    private final Vec2 localAnchorB = new Vec2();

    private float maxLength;

    private float length;

    private float impulse;

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

    private LimitState state;

    protected RopeJoint(IWorldPool worldPool, RopeJointDef def)
    {
        super(worldPool, def);
        localAnchorA.set(def.localAnchorA);
        localAnchorB.set(def.localAnchorB);
        maxLength = def.maxLength;
        mass = 0.0f;
        impulse = 0.0f;
        state = LimitState.INACTIVE;
        length = 0.0f;
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
        final Vec2 temp = pool.popVec2();
        qA.set(aA);
        qB.set(aB);
        // Compute the effective masses.
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA),
                rA);
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB),
                rB);
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);
        length = u.length();
        float C = length - maxLength;
        if (C > 0.0f)
        {
            state = LimitState.AT_UPPER;
        }
        else
        {
            state = LimitState.INACTIVE;
        }
        if (length > Settings.linearSlop)
        {
            u.mulLocal(1.0f / length);
        }
        else
        {
            u.setZero();
            mass = 0.0f;
            impulse = 0.0f;
            pool.pushRot(2);
            pool.pushVec2(1);
            return;
        }
        // Compute effective mass.
        float crA = Vec2.cross(rA, u);
        float crB = Vec2.cross(rB, u);
        float invMass = invMassA + invIA * crA * crA + invMassB
                + invIB * crB * crB;
        mass = invMass != 0.0f ? 1.0f / invMass : 0.0f;
        if (data.step.warmStarting)
        {
            // Scale the impulse to support a variable time step.
            impulse *= data.step.dtRatio;
            float Px = impulse * u.x;
            float Py = impulse * u.y;
            vA.x -= invMassA * Px;
            vA.y -= invMassA * Py;
            wA -= invIA * (rA.x * Py - rA.y * Px);
            vB.x += invMassB * Px;
            vB.y += invMassB * Py;
            wB += invIB * (rB.x * Py - rB.y * Px);
        }
        else
        {
            impulse = 0.0f;
        }
        pool.pushRot(2);
        pool.pushVec2(1);
        // data.velocities[m_indexA].v = vA;
        data.velocities[indexA].w = wA;
        // data.velocities[m_indexB].v = vB;
        data.velocities[indexB].w = wB;
    }

    @Override
    public void solveVelocityConstraints(final SolverData data)
    {
        Vec2 vA = data.velocities[indexA].v;
        float wA = data.velocities[indexA].w;
        Vec2 vB = data.velocities[indexB].v;
        float wB = data.velocities[indexB].w;
        // Cdot = dot(u, v + cross(w, r))
        Vec2 vpA = pool.popVec2();
        Vec2 vpB = pool.popVec2();
        Vec2 temp = pool.popVec2();
        Vec2.crossToOutUnsafe(wA, rA, vpA);
        vpA.addLocal(vA);
        Vec2.crossToOutUnsafe(wB, rB, vpB);
        vpB.addLocal(vB);
        float C = length - maxLength;
        float Cdot = Vec2.dot(u, temp.set(vpB).subLocal(vpA));
        // Predictive constraint.
        if (C < 0.0f)
        {
            Cdot += data.step.inv_dt * C;
        }
        float impulse = -mass * Cdot;
        float oldImpulse = this.impulse;
        this.impulse = MathUtils.min(0.0f, this.impulse + impulse);
        impulse = this.impulse - oldImpulse;
        float Px = impulse * u.x;
        float Py = impulse * u.y;
        vA.x -= invMassA * Px;
        vA.y -= invMassA * Py;
        wA -= invIA * (rA.x * Py - rA.y * Px);
        vB.x += invMassB * Px;
        vB.y += invMassB * Py;
        wB += invIB * (rB.x * Py - rB.y * Px);
        pool.pushVec2(3);
        // data.velocities[m_indexA].v = vA;
        data.velocities[indexA].w = wA;
        // data.velocities[m_indexB].v = vB;
        data.velocities[indexB].w = wB;
    }

    @Override
    public boolean solvePositionConstraints(final SolverData data)
    {
        Vec2 cA = data.positions[indexA].c;
        float aA = data.positions[indexA].a;
        Vec2 cB = data.positions[indexB].c;
        float aB = data.positions[indexB].a;
        final Rot qA = pool.popRot();
        final Rot qB = pool.popRot();
        final Vec2 u = pool.popVec2();
        final Vec2 rA = pool.popVec2();
        final Vec2 rB = pool.popVec2();
        final Vec2 temp = pool.popVec2();
        qA.set(aA);
        qB.set(aB);
        // Compute the effective masses.
        Rot.mulToOutUnsafe(qA, temp.set(localAnchorA).subLocal(localCenterA),
                rA);
        Rot.mulToOutUnsafe(qB, temp.set(localAnchorB).subLocal(localCenterB),
                rB);
        u.set(cB).addLocal(rB).subLocal(cA).subLocal(rA);
        float length = u.normalize();
        float C = length - maxLength;
        C = MathUtils.clamp(C, 0.0f, Settings.maxLinearCorrection);
        float impulse = -mass * C;
        float Px = impulse * u.x;
        float Py = impulse * u.y;
        cA.x -= invMassA * Px;
        cA.y -= invMassA * Py;
        aA -= invIA * (rA.x * Py - rA.y * Px);
        cB.x += invMassB * Px;
        cB.y += invMassB * Py;
        aB += invIB * (rB.x * Py - rB.y * Px);
        pool.pushRot(2);
        pool.pushVec2(4);
        // data.positions[m_indexA].c = cA;
        data.positions[indexA].a = aA;
        // data.positions[m_indexB].c = cB;
        data.positions[indexB].a = aB;
        return length - maxLength < Settings.linearSlop;
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

    @Override
    public void getReactionForce(float inv_dt, Vec2 argOut)
    {
        argOut.set(u).mulLocal(inv_dt).mulLocal(impulse);
    }

    @Override
    public float getReactionTorque(float inv_dt)
    {
        return 0f;
    }

    public Vec2 getLocalAnchorA()
    {
        return localAnchorA;
    }

    public Vec2 getLocalAnchorB()
    {
        return localAnchorB;
    }

    public float getMaxLength()
    {
        return maxLength;
    }

    public void setMaxLength(float maxLength)
    {
        this.maxLength = maxLength;
    }

    public LimitState getLimitState()
    {
        return state;
    }
}
