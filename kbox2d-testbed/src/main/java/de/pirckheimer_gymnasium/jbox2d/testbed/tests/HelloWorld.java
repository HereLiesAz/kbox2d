package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.FixtureDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.World;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * https://box2d.org/documentation/md__d_1__git_hub_box2d_docs_hello.html
 */
public class HelloWorld extends TestbedTest
{

    @Override
    public void initTest(boolean deserialized)
    {
        // Static Body
        Vec2 gravity = new Vec2(0, -10);
        World world = getWorld();
        world.setGravity(gravity);
        BodyDef groundBodyDef = new BodyDef();
        groundBodyDef.position.set(0, -10);
        Body groundBody = world.createBody(groundBodyDef);
        PolygonShape groundBox = new PolygonShape();
        groundBox.setAsBox(50, 10);
        groundBody.createFixture(groundBox, 0);
        // Dynamic Body
        BodyDef bodyDef = new BodyDef();
        bodyDef.type = BodyType.DYNAMIC;
        bodyDef.position.set(0, 4);
        Body body = world.createBody(bodyDef);
        PolygonShape dynamicBox = new PolygonShape();
        dynamicBox.setAsBox(1, 1);
        FixtureDef fixtureDef = new FixtureDef();
        fixtureDef.shape = dynamicBox;
        fixtureDef.density = 1;
        fixtureDef.friction = 0.3f;
        body.createFixture(fixtureDef);

    }

    @Override
    public String getTestName()
    {
        return "Hello World";
    }
}
