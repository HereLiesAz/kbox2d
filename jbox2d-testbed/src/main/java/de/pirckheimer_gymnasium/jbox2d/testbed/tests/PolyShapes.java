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
 * Created at 1:41:40 PM Jan 23, 2011
 */
package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.callbacks.DebugDraw;
import de.pirckheimer_gymnasium.jbox2d.callbacks.QueryCallback;
import de.pirckheimer_gymnasium.jbox2d.collision.AABB;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.CircleShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.Shape;
import de.pirckheimer_gymnasium.jbox2d.common.Color3f;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Transform;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Fixture;
import de.pirckheimer_gymnasium.jbox2d.dynamics.FixtureDef;
import de.pirckheimer_gymnasium.jbox2d.pooling.IWorldPool;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class PolyShapes extends TestbedTest
{
    int maxBodies = 256;

    int bodyIndex;

    Body[] bodies = new Body[maxBodies];

    PolygonShape[] polygons = new PolygonShape[4];

    CircleShape circle;

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
        {
            Vec2 vertices[] = new Vec2[3];
            vertices[0] = new Vec2(-0.5f, 0.0f);
            vertices[1] = new Vec2(0.5f, 0.0f);
            vertices[2] = new Vec2(0.0f, 1.5f);
            polygons[0] = new PolygonShape();
            polygons[0].set(vertices, 3);
        }
        {
            Vec2 vertices[] = new Vec2[3];
            vertices[0] = new Vec2(-0.1f, 0.0f);
            vertices[1] = new Vec2(0.1f, 0.0f);
            vertices[2] = new Vec2(0.0f, 1.5f);
            polygons[1] = new PolygonShape();
            polygons[1].set(vertices, 3);
        }
        {
            float w = 1.0f;
            float b = w / (2.0f + MathUtils.sqrt(2.0f));
            float s = MathUtils.sqrt(2.0f) * b;
            Vec2 vertices[] = new Vec2[8];
            vertices[0] = new Vec2(0.5f * s, 0.0f);
            vertices[1] = new Vec2(0.5f * w, b);
            vertices[2] = new Vec2(0.5f * w, b + s);
            vertices[3] = new Vec2(0.5f * s, w);
            vertices[4] = new Vec2(-0.5f * s, w);
            vertices[5] = new Vec2(-0.5f * w, b + s);
            vertices[6] = new Vec2(-0.5f * w, b);
            vertices[7] = new Vec2(-0.5f * s, 0.0f);
            polygons[2] = new PolygonShape();
            polygons[2].set(vertices, 8);
        }
        {
            polygons[3] = new PolygonShape();
            polygons[3].setAsBox(0.5f, 0.5f);
        }
        {
            circle = new CircleShape();
            circle.radius = 0.5f;
        }
        bodyIndex = 0;
    }

    void Create(int index)
    {
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        float x = MathUtils.randomFloat(-2.0f, 2.0f);
        bd.position.set(x, 10.0f);
        bd.angle = MathUtils.randomFloat(-MathUtils.PI, MathUtils.PI);
        if (index == 4)
        {
            bd.angularDamping = 0.02f;
        }
        bodies[bodyIndex] = getWorld().createBody(bd);
        if (index < 4)
        {
            FixtureDef fd = new FixtureDef();
            fd.shape = polygons[index];
            fd.density = 1.0f;
            fd.friction = 0.3f;
            bodies[bodyIndex].createFixture(fd);
        }
        else
        {
            FixtureDef fd = new FixtureDef();
            fd.shape = circle;
            fd.density = 1.0f;
            fd.friction = 0.3f;
            bodies[bodyIndex].createFixture(fd);
        }
        bodyIndex = (bodyIndex + 1) % maxBodies;
    }

    void DestroyBody()
    {
        for (int i = 0; i < maxBodies; ++i)
        {
            if (bodies[i] != null)
            {
                getWorld().destroyBody(bodies[i]);
                bodies[i] = null;
                return;
            }
        }
    }

    @Override
    public void keyPressed(char key, int argKeyCode)
    {
        switch (key)
        {
        case '1':
        case '2':
        case '3':
        case '4':
        case '5':
            Create(key - '1');
            break;

        case 'a':
            for (int i = 0; i < maxBodies; i += 2)
            {
                if (bodies[i] != null)
                {
                    boolean active = bodies[i].isActive();
                    bodies[i].setActive(!active);
                }
            }
            break;

        case 'd':
            DestroyBody();
            break;
        }
    }

    /**
     * @see de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest#step(de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings)
     */
    @Override
    public void step(TestbedSettings settings)
    {
        super.step(settings);
        PolyShapesCallback callback = new PolyShapesCallback(
                getWorld().getPool());
        callback.circle.radius = 2.0f;
        callback.circle.p.set(0.0f, 2.1f);
        callback.transform.setIdentity();
        callback.debugDraw = getDebugDraw();
        AABB aabb = new AABB();
        callback.circle.computeAABB(aabb, callback.transform, 0);
        getWorld().queryAABB(callback, aabb);
        Color3f color = new Color3f(0.4f, 0.7f, 0.8f);
        getDebugDraw().drawCircle(callback.circle.p, callback.circle.radius,
                color);
        addTextLine("Press 1-5 to drop stuff");
        addTextLine("Press 'a' to (de)activate some bodies");
        addTextLine("Press 'd' to destroy a body");
        addTextLine("Up to 30 bodies in the target circle are highlighted");
    }

    /**
     * @see de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest#getTestName()
     */
    @Override
    public String getTestName()
    {
        return "PolyShapes";
    }
}

/**
 * This callback is called by b2World::QueryAABB. We find all the fixtures that
 * overlap an AABB. Of those, we use b2TestOverlap to determine which fixtures
 * overlap a circle. Up to 30 overlapped fixtures will be highlighted with a
 * yellow border.
 *
 * @author Daniel Murphy
 */
class PolyShapesCallback implements QueryCallback
{
    int maxCount = 30;

    CircleShape circle = new CircleShape();

    Transform transform = new Transform();

    DebugDraw debugDraw;

    int count;

    IWorldPool p;

    public PolyShapesCallback(IWorldPool argWorld)
    {
        count = 0;
        p = argWorld;
    }

    void DrawFixture(Fixture fixture)
    {
        Color3f color = new Color3f(0.95f, 0.95f, 0.6f);
        final Transform xf = fixture.getBody().getTransform();
        switch (fixture.getType())
        {
        case CIRCLE:
        {
            CircleShape circle = (CircleShape) fixture.getShape();
            Vec2 center = Transform.mul(xf, circle.p);
            float radius = circle.radius;
            debugDraw.drawCircle(center, radius, color);
        }
            break;

        case POLYGON:
        {
            PolygonShape poly = (PolygonShape) fixture.getShape();
            int vertexCount = poly.count;
            assert (vertexCount <= Settings.maxPolygonVertices);
            Vec2 vertices[] = new Vec2[Settings.maxPolygonVertices];
            for (int i = 0; i < vertexCount; ++i)
            {
                vertices[i] = Transform.mul(xf, poly.vertices[i]);
            }
            debugDraw.drawPolygon(vertices, vertexCount, color);
        }
            break;

        default:
            break;
        }
    }

    public boolean reportFixture(Fixture fixture)
    {
        if (count == maxCount)
        {
            return false;
        }
        Body body = fixture.getBody();
        Shape shape = fixture.getShape();
        boolean overlap = p.getCollision().testOverlap(shape, 0, circle, 0,
                body.getTransform(), transform);
        if (overlap)
        {
            DrawFixture(fixture);
            ++count;
        }
        return true;
    }
}
