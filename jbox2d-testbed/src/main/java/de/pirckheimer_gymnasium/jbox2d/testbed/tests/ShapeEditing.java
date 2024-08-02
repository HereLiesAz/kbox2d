/*******************************************************************************
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
 ******************************************************************************/
/**
 * Created at 2:04:52 PM Jan 23, 2011
 */
package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.CircleShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Fixture;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class ShapeEditing extends TestbedTest
{
    Body body;

    Fixture fixture1;

    Fixture fixture2;

    @Override
    public void initTest(boolean argDeserialized)
    {
        {
            BodyDef bd = new BodyDef();
            Body ground = getWorld().createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(-40.0f, 0.0f), new Vec2(40.0f, 0.0f));
            ground.createFixture(shape, 0.0f);
        }
        BodyDef bd = new BodyDef();
        bd.type = BodyType.DYNAMIC;
        bd.position.set(0.0f, 10.0f);
        body = getWorld().createBody(bd);
        PolygonShape shape = new PolygonShape();
        shape.setAsBox(4.0f, 4.0f, new Vec2(0.0f, 0.0f), 0.0f);
        fixture1 = body.createFixture(shape, 10.0f);
        fixture2 = null;
    }

    @Override
    public void keyPressed(char key, int argKeyCode)
    {
        switch (key)
        {
        case 'c':
            if (fixture2 == null)
            {
                CircleShape shape = new CircleShape();
                shape.radius = 3.0f;
                shape.p.set(0.5f, -4.0f);
                fixture2 = body.createFixture(shape, 10.0f);
                body.setAwake(true);
            }
            break;

        case 'd':
            if (fixture2 != null)
            {
                body.destroyFixture(fixture2);
                fixture2 = null;
                body.setAwake(true);
            }
            break;
        }
    }

    @Override
    public void step(TestbedSettings settings)
    {
        super.step(settings);
        addTextLine("Press: (c) create a shape, (d) destroy a shape.");
    }

    @Override
    public String getTestName()
    {
        return "Shape Editing";
    }
}
