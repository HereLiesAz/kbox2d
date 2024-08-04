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

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.CircleShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.Body;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.BodyType;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.GearJoint;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.GearJointDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.Joint;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.PrismaticJoint;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.PrismaticJointDef;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.RevoluteJoint;
import de.pirckheimer_gymnasium.jbox2d.dynamics.joints.RevoluteJointDef;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 */
public class Gears extends TestbedTest
{
    RevoluteJoint joint1;

    RevoluteJoint joint2;

    PrismaticJoint joint3;

    GearJoint joint4;

    GearJoint joint5;

    @Override
    public void initTest(boolean argDeserialized)
    {
        Body ground = null;
        {
            BodyDef bd = new BodyDef();
            ground = getWorld().createBody(bd);
            EdgeShape shape = new EdgeShape();
            shape.set(new Vec2(50.0f, 0.0f), new Vec2(-50.0f, 0.0f));
            ground.createFixture(shape, 0.0f);
        }
        {
            CircleShape circle1 = new CircleShape();
            circle1.radius = 1.0f;
            PolygonShape box = new PolygonShape();
            box.setAsBox(0.5f, 5.0f);
            CircleShape circle2 = new CircleShape();
            circle2.radius = 2.0f;
            BodyDef bd1 = new BodyDef();
            bd1.type = BodyType.STATIC;
            bd1.position.set(10.0f, 9.0f);
            Body body1 = world.createBody(bd1);
            body1.createFixture(circle1, 5.0f);
            BodyDef bd2 = new BodyDef();
            bd2.type = BodyType.DYNAMIC;
            bd2.position.set(10.0f, 8.0f);
            Body body2 = world.createBody(bd2);
            body2.createFixture(box, 5.0f);
            BodyDef bd3 = new BodyDef();
            bd3.type = BodyType.DYNAMIC;
            bd3.position.set(10.0f, 6.0f);
            Body body3 = world.createBody(bd3);
            body3.createFixture(circle2, 5.0f);
            RevoluteJointDef jd1 = new RevoluteJointDef();
            jd1.initialize(body2, body1, bd1.position);
            Joint joint1 = world.createJoint(jd1);
            RevoluteJointDef jd2 = new RevoluteJointDef();
            jd2.initialize(body2, body3, bd3.position);
            Joint joint2 = world.createJoint(jd2);
            GearJointDef jd4 = new GearJointDef();
            jd4.bodyA = body1;
            jd4.bodyB = body3;
            jd4.joint1 = joint1;
            jd4.joint2 = joint2;
            jd4.ratio = circle2.radius / circle1.radius;
            world.createJoint(jd4);
        }
        {
            CircleShape circle1 = new CircleShape();
            circle1.radius = 1.0f;
            CircleShape circle2 = new CircleShape();
            circle2.radius = 2.0f;
            PolygonShape box = new PolygonShape();
            box.setAsBox(0.5f, 5.0f);
            BodyDef bd1 = new BodyDef();
            bd1.type = BodyType.DYNAMIC;
            bd1.position.set(-3.0f, 12.0f);
            Body body1 = world.createBody(bd1);
            body1.createFixture(circle1, 5.0f);
            RevoluteJointDef jd1 = new RevoluteJointDef();
            jd1.bodyA = ground;
            jd1.bodyB = body1;
            ground.getLocalPointToOut(bd1.position, jd1.localAnchorA);
            body1.getLocalPointToOut(bd1.position, jd1.localAnchorB);
            jd1.referenceAngle = body1.getAngle() - ground.getAngle();
            joint1 = (RevoluteJoint) world.createJoint(jd1);
            BodyDef bd2 = new BodyDef();
            bd2.type = BodyType.DYNAMIC;
            bd2.position.set(0.0f, 12.0f);
            Body body2 = world.createBody(bd2);
            body2.createFixture(circle2, 5.0f);
            RevoluteJointDef jd2 = new RevoluteJointDef();
            jd2.initialize(ground, body2, bd2.position);
            joint2 = (RevoluteJoint) world.createJoint(jd2);
            BodyDef bd3 = new BodyDef();
            bd3.type = BodyType.DYNAMIC;
            bd3.position.set(2.5f, 12.0f);
            Body body3 = world.createBody(bd3);
            body3.createFixture(box, 5.0f);
            PrismaticJointDef jd3 = new PrismaticJointDef();
            jd3.initialize(ground, body3, bd3.position, new Vec2(0.0f, 1.0f));
            jd3.lowerTranslation = -5.0f;
            jd3.upperTranslation = 5.0f;
            jd3.enableLimit = true;
            joint3 = (PrismaticJoint) world.createJoint(jd3);
            GearJointDef jd4 = new GearJointDef();
            jd4.bodyA = body1;
            jd4.bodyB = body2;
            jd4.joint1 = joint1;
            jd4.joint2 = joint2;
            jd4.ratio = circle2.radius / circle1.radius;
            joint4 = (GearJoint) world.createJoint(jd4);
            GearJointDef jd5 = new GearJointDef();
            jd5.bodyA = body2;
            jd5.bodyB = body3;
            jd5.joint1 = joint2;
            jd5.joint2 = joint3;
            jd5.ratio = 1f / circle2.radius;
            joint5 = (GearJoint) world.createJoint(jd5);
        }
    }

    @Override
    public void step(TestbedSettings settings)
    {
        super.step(settings);
        float ratio, value;
        ratio = joint4.getRatio();
        value = joint1.getJointAngle() + ratio * joint2.getJointAngle();
        addTextLine("theta1 + " + ratio + " * theta2 = " + value);
        ratio = joint5.getRatio();
        value = joint2.getJointAngle() + ratio * joint3.getJointTranslation();
        addTextLine("theta2 + " + ratio + " * delta = " + value);
    }

    @Override
    public String getTestName()
    {
        return "Gears";
    }
}
