package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.ChainShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleGroupDef;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleType;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

public class LiquidTimer extends TestbedTest
{
    @Override
    public void initTest(boolean deserialized)
    {
        {
            BodyDef bd = new BodyDef();
            Body ground = world.createBody(bd);
            ChainShape shape = new ChainShape();
            final Vec2[] vertices = new Vec2[] { new Vec2(-20, 0),
                    new Vec2(20, 0), new Vec2(20, 40), new Vec2(-20, 40) };
            shape.createLoop(vertices, 4);
            ground.createFixture(shape, 0.0f);
        }
        world.setParticleRadius(0.15f);
        {
            PolygonShape shape = new PolygonShape();
            shape.setAsBox(20, 4, new Vec2(0, 36), 0);
            ParticleGroupDef pd = new ParticleGroupDef();
            pd.flags = ParticleType.tensileParticle
                    | ParticleType.viscousParticle;
            pd.shape = shape;
            world.createParticleGroup(pd);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-20, 32), new Vec2(-12, 32));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-11, 32), new Vec2(20, 32));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-12, 32), new Vec2(-12, 28));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-11, 32), new Vec2(-11, 28));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-16, 24), new Vec2(8, 20));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(16, 16), new Vec2(-8, 12));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-12, 8), new Vec2(-12, 0));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-4, 8), new Vec2(-4, 0));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(4, 8), new Vec2(4, 0));
            body.createFixture(shape, 0.1f);
        }
        {
            BodyDef bd = new BodyDef();
            Body body = world.createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(12, 8), new Vec2(12, 0));
            body.createFixture(shape, 0.1f);
        }
    }

    @Override
    public String getTestName()
    {
        return "Liquid Timer";
    }
}
