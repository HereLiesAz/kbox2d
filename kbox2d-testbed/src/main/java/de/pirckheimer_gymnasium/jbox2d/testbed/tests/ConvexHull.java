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

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Color3f;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

/**
 * @author Daniel Murphy
 *
 * @repolink https://github.com/google/liquidfun/blob/master/liquidfun/Box2D/Testbed/Tests/ConvexHull.h
 */
public class ConvexHull extends TestbedTest
{
    private final int count = Settings.maxPolygonVertices;

    private boolean auto = false;

    private final Vec2[] memberPoints = new Vec2[Settings.maxPolygonVertices];

    private int memberCount;

    @Override
    public void initTest(boolean deserialized)
    {
        if (deserialized)
        {
            return;
        }
        generate();
    }

    void generate()
    {
        Vec2 lowerBound = new Vec2(-8f, -8f);
        Vec2 upperBound = new Vec2(8f, 8f);
        for (int i = 0; i < count; i++)
        {
            float x = MathUtils.randomFloat(-8, 8);
            float y = MathUtils.randomFloat(-8, 8);
            Vec2 v = new Vec2(x, y);
            MathUtils.clampToOut(v, lowerBound, upperBound, v);
            memberPoints[i] = v;
        }
        memberCount = count;
    }

    public void keyPressed(char argKeyChar, int argKeyCode)
    {
        if (argKeyChar == 'g')
        {
            generate();
        }
        else if (argKeyChar == 'a')
        {
            auto = !auto;
        }
    }

    PolygonShape shape = new PolygonShape();

    Color3f color = new Color3f(.9f, .9f, .9f);

    Color3f color2 = new Color3f(.9f, .5f, .5f);

    @Override
    public synchronized void step(TestbedSettings settings)
    {
        super.step(settings);
        shape.set(memberPoints, memberCount);
        addTextLine("Press g to generate a new random convex hull");
        getDebugDraw().drawPolygon(shape.vertices, shape.count, color);
        for (int i = 0; i < memberCount; ++i)
        {
            getDebugDraw().drawPoint(memberPoints[i], 2.0f, color2);
            getDebugDraw().drawString(
                    memberPoints[i].add(new Vec2(0.05f, 0.05f)), i + "",
                    Color3f.WHITE);
        }
        assert (shape.validate());
        if (auto)
        {
            generate();
        }
    }

    @Override
    public String getTestName()
    {
        return "Convex Hull";
    }
}
