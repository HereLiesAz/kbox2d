package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.RevoluteJoint;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.RevoluteJointDef;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleGroupDef;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

public class WaveMachine extends TestbedTest
{
    RevoluteJoint joint;

    float time;

    @Override
    public void step(TestbedSettings settings)
    {
        super.step(settings);
        float hz = settings.getSetting(TestbedSettings.Hz).value;
        if (hz > 0)
        {
            time += 1 / hz;
        }
        joint.setMotorSpeed(0.05f * MathUtils.cos(time) * MathUtils.PI);
    }

    @Override
    public void initTest(boolean deserialized)
    {
        {
            BodyDef bd = new BodyDef();
            bd.type = BodyType.DYNAMIC;
            bd.allowSleep = false;
            bd.position.set(0.0f, 10.0f);
            Body body = world.createBody(bd);
            PolygonShape shape = new PolygonShape();
            shape.setAsBox(0.5f, 10.0f, new Vec2(20.0f, 0.0f), 0.0f);
            body.createFixture(shape, 5.0f);
            shape.setAsBox(0.5f, 10.0f, new Vec2(-20.0f, 0.0f), 0.0f);
            body.createFixture(shape, 5.0f);
            shape.setAsBox(20.0f, 0.5f, new Vec2(0.0f, 10.0f), 0.0f);
            body.createFixture(shape, 5.0f);
            shape.setAsBox(20.0f, 0.5f, new Vec2(0.0f, -10.0f), 0.0f);
            body.createFixture(shape, 5.0f);
            RevoluteJointDef jd = new RevoluteJointDef();
            jd.bodyA = getGroundBody();
            jd.bodyB = body;
            jd.localAnchorA.set(0.0f, 10.0f);
            jd.localAnchorB.set(0.0f, 0.0f);
            jd.referenceAngle = 0.0f;
            jd.motorSpeed = 0.05f * MathUtils.PI;
            jd.maxMotorTorque = 1e7f;
            jd.enableMotor = true;
            joint = (RevoluteJoint) world.createJoint(jd);
        }
        world.setParticleRadius(0.15f);
        world.setParticleDamping(0.2f);
        {
            ParticleGroupDef pd = new ParticleGroupDef();
            pd.flags = 0;
            PolygonShape shape = new PolygonShape();
            shape.setAsBox(9.0f, 9.0f, new Vec2(0.0f, 10.0f), 0.0f);
            pd.shape = shape;
            world.createParticleGroup(pd);
        }
        time = 0;
    }

    @Override
    public String getTestName()
    {
        return "Wave Machine";
    }
}
