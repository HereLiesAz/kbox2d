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
/**
 * Created at 5:18:10 AM Jan 14, 2011
 */
package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.callbacks.ContactImpulse;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Fixture;
import de.pirckheimer_gymnasium.jbox2d.dynamics.contacts.Contact;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Breakable extends TestbedTest
{
    Body body1;

    Vec2 velocity = new Vec2();

    float angularVelocity;

    PolygonShape shape1;

    PolygonShape shape2;

    Fixture piece1;

    Fixture piece2;

    boolean broke;

    boolean doBreak;

    @Override
    public void initTest(boolean argDeserialized)
    {
        // Ground body
        {
            BodyDef bd = new BodyDef();
            Body ground = getWorld().createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
            ground.createFixture(shape, 0.0f);
        }
        // Breakable dynamic body
        {
            BodyDef bd = new BodyDef();
            bd.type = BodyType.DYNAMIC;
            bd.position.set(0.0f, 40.0f);
            bd.angle = 0.25f * MathUtils.PI;
            body1 = getWorld().createBody(bd);
            shape1 = new PolygonShape();
            shape1.setAsBox(0.5f, 0.5f, new Vec2(-0.5f, 0.0f), 0.0f);
            piece1 = body1.createFixture(shape1, 1.0f);
            shape2 = new PolygonShape();
            shape2.setAsBox(0.5f, 0.5f, new Vec2(0.5f, 0.0f), 0.0f);
            piece2 = body1.createFixture(shape2, 1.0f);
        }
        doBreak = false;
        broke = false;
    }

    @Override
    public void postSolve(Contact contact, ContactImpulse impulse)
    {
        if (broke)
        {
            // The body already broke.
            return;
        }
        // Should the body break?
        int count = contact.getManifold().pointCount;
        float maxImpulse = 0.0f;
        for (int i = 0; i < count; ++i)
        {
            maxImpulse = MathUtils.max(maxImpulse, impulse.normalImpulses[i]);
        }
        if (maxImpulse > 40.0f)
        {
            // Flag the body for breaking.
            doBreak = true;
        }
    }

    void Break()
    {
        // Create two bodies from one.
        Body body1 = piece1.getBody();
        Vec2 center = body1.getWorldCenter();
        body1.destroyFixture(piece2);
        piece2 = null;
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position = body1.getPosition();
        bd.angle = body1.getAngle();
        Body body2 = getWorld().createBody(bd);
        piece2 = body2.createFixture(shape2, 1.0f);
        // Compute consistent velocities for new bodies based on
        // cached velocity.
        Vec2 center1 = body1.getWorldCenter();
        Vec2 center2 = body2.getWorldCenter();
        Vec2 velocity1 = velocity
                .add(Vec2.cross(angularVelocity, center1.sub(center)));
        Vec2 velocity2 = velocity
                .add(Vec2.cross(angularVelocity, center2.sub(center)));
        body1.setAngularVelocity(angularVelocity);
        body1.setLinearVelocity(velocity1);
        body2.setAngularVelocity(angularVelocity);
        body2.setLinearVelocity(velocity2);
    }

    @Override
    public void step(TestbedSettings settings)
    {
        super.step(settings);
        if (doBreak)
        {
            Break();
            broke = true;
            doBreak = false;
        }
        // Cache velocities to improve movement on breakage.
        if (broke == false)
        {
            velocity.set(body1.getLinearVelocity());
            angularVelocity = body1.getAngularVelocity();
        }
    }

    @Override
    public String getTestName()
    {
        return "Breakable";
    }
}
