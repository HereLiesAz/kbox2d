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
package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.collision.Manifold;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.CircleShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Fixture;
import de.pirckheimer_gymnasium.jbox2d.dynamics.contacts.Contact;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 *
 * @repolink https://github.com/google/liquidfun/blob/master/liquidfun/Box2D/Testbed/Tests/OneSidedPlatform.h
 */
public class OneSidedTest extends TestbedTest
{
    private static final long PLATFORM_TAG = 10;

    private static final long CHARACTER_TAG = 11;

    enum State
    {
        unknown, above, below,
    };

    float radius, top, bottom;

    State state;

    Fixture platform;

    Fixture character;

    @Override
    public Long getTag(Fixture fixture)
    {
        if (fixture == platform)
            return PLATFORM_TAG;
        if (fixture == character)
            return CHARACTER_TAG;
        return super.getTag(fixture);
    }

    @Override
    public void processFixture(Fixture fixture, Long tag)
    {
        if (tag == PLATFORM_TAG)
        {
            platform = fixture;
        }
        else if (tag == CHARACTER_TAG)
        {
            character = fixture;
        }
        else
        {
            super.processFixture(fixture, tag);
        }
    }

    @Override
    public boolean isSaveLoadEnabled()
    {
        return true;
    }

    @Override
    public String getTestName()
    {
        return "One Sided";
    }

    @Override
    public void initTest(boolean deserialized)
    {
        state = State.unknown;
        if (deserialized)
        {
            return;
        }
        // Ground
        {
            BodyDef bd = new BodyDef();
            Body ground = getWorld().createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-20.0f, 0.0f), new Vec2(20.0f, 0.0f));
            ground.createFixture(shape, 0.0f);
        }
        // Platform
        {
            BodyDef bd = new BodyDef();
            bd.position.set(0.0f, 10.0f);
            Body body = getWorld().createBody(bd);
            PolygonShape shape = new PolygonShape();
            shape.setAsBox(3.0f, 0.5f);
            platform = body.createFixture(shape, 0.0f);
            bottom = 10.0f - 0.5f;
            top = 10.0f + 0.5f;
        }
        // Actor
        {
            BodyDef bd = new BodyDef();
            bd.type = BodyType.DYNAMIC;
            bd.position.set(0.0f, 12.0f);
            Body body = getWorld().createBody(bd);
            radius = 0.5f;
            CircleShape shape = new CircleShape();
            shape.radius = radius;
            character = body.createFixture(shape, 20.0f);
            body.setLinearVelocity(new Vec2(0.0f, -50.0f));
            state = State.unknown;
        }
    }

    @Override
    public void preSolve(Contact contact, Manifold oldManifold)
    {
        super.preSolve(contact, oldManifold);
        Fixture fixtureA = contact.getFixtureA();
        Fixture fixtureB = contact.getFixtureB();
        if (fixtureA != platform && fixtureA != character)
        {
            return;
        }
        if (fixtureB != character && fixtureB != character)
        {
            return;
        }
        Vec2 position = character.getBody().getPosition();
        if (position.y < top + radius - 3.0f * Settings.linearSlop)
        {
            contact.setEnabled(false);
        }
    }
}
