package de.pirckheimer_gymnasium.jbox2d.particle;

import de.pirckheimer_gymnasium.jbox2d.common.Transform;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;

public class ParticleGroup
{
    ParticleSystem system;

    int firstIndex;

    int lastIndex;

    int groupFlags;

    float strength;

    ParticleGroup prev;

    ParticleGroup next;

    int timestamp;

    float mass;

    float inertia;

    final Vec2 center = new Vec2();

    final Vec2 linearVelocity = new Vec2();

    float angularVelocity;

    final Transform transform = new Transform();

    boolean destroyAutomatically;

    boolean toBeDestroyed;

    boolean toBeSplit;

    Object userData;

    public ParticleGroup()
    {
        // m_system = null;
        firstIndex = 0;
        lastIndex = 0;
        groupFlags = 0;
        strength = 1.0f;
        timestamp = -1;
        mass = 0;
        inertia = 0;
        angularVelocity = 0;
        transform.setIdentity();
        destroyAutomatically = true;
        toBeDestroyed = false;
        toBeSplit = false;
    }

    public ParticleGroup getNext()
    {
        return next;
    }

    public int getParticleCount()
    {
        return lastIndex - firstIndex;
    }

    public int getBufferIndex()
    {
        return firstIndex;
    }

    public int getGroupFlags()
    {
        return groupFlags;
    }

    public void setGroupFlags(int flags)
    {
        groupFlags = flags;
    }

    public float getMass()
    {
        updateStatistics();
        return mass;
    }

    public float getInertia()
    {
        updateStatistics();
        return inertia;
    }

    public Vec2 getCenter()
    {
        updateStatistics();
        return center;
    }

    public Vec2 getLinearVelocity()
    {
        updateStatistics();
        return linearVelocity;
    }

    public float getAngularVelocity()
    {
        updateStatistics();
        return angularVelocity;
    }

    public Transform getTransform()
    {
        return transform;
    }

    public Vec2 getPosition()
    {
        return transform.p;
    }

    public float getAngle()
    {
        return transform.q.getAngle();
    }

    public Object getUserData()
    {
        return userData;
    }

    public void setUserData(Object data)
    {
        userData = data;
    }

    public void updateStatistics()
    {
        if (timestamp != system.timestamp)
        {
            float m = system.getParticleMass();
            mass = 0;
            center.setZero();
            linearVelocity.setZero();
            for (int i = firstIndex; i < lastIndex; i++)
            {
                mass += m;
                Vec2 pos = system.positionBuffer.data[i];
                center.x += m * pos.x;
                center.y += m * pos.y;
                Vec2 vel = system.velocityBuffer.data[i];
                linearVelocity.x += m * vel.x;
                linearVelocity.y += m * vel.y;
            }
            if (mass > 0)
            {
                center.x *= 1 / mass;
                center.y *= 1 / mass;
                linearVelocity.x *= 1 / mass;
                linearVelocity.y *= 1 / mass;
            }
            inertia = 0;
            angularVelocity = 0;
            for (int i = firstIndex; i < lastIndex; i++)
            {
                Vec2 pos = system.positionBuffer.data[i];
                Vec2 vel = system.velocityBuffer.data[i];
                float px = pos.x - center.x;
                float py = pos.y - center.y;
                float vx = vel.x - linearVelocity.x;
                float vy = vel.y - linearVelocity.y;
                inertia += m * (px * px + py * py);
                angularVelocity += m * (px * vy - py * vx);
            }
            if (inertia > 0)
            {
                angularVelocity *= 1 / inertia;
            }
            timestamp = system.timestamp;
        }
    }
}
