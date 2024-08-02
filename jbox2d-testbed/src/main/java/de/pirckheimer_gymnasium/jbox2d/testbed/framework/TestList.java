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
package de.pirckheimer_gymnasium.jbox2d.testbed.framework;

import de.pirckheimer_gymnasium.jbox2d.testbed.tests.ApplyForce;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.BlobTest4;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.BodyTypes;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Breakable;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.BulletTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Cantilever;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Car;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Chain;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.CharacterCollision;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.CircleStress;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.CollisionFiltering;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.CollisionProcessing;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.CompoundShapes;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.ConfinedTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.ContinuousTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.ConvexHull;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.ConveyorBelt;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.DamBreak;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.DistanceTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.DominoTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.DominoTower;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.DrawingParticles;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.DynamicTreeTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.EdgeShapes;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.FixedPendulumTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.FreePendulumTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Gears;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.LiquidTimer;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.MotorTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.OneSidedTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Particles;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.PistonTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.PolyShapes;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.PrismaticTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Pulleys;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.PyramidTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.RayCastTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.RevoluteTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.RopeTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.SensorTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.ShapeEditing;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.SliderCrankTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.SphereStack;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.TheoJansen;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Tumbler;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.VaryingFrictionTest;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.VaryingRestitution;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.VerticalStack;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.WaveMachine;
import de.pirckheimer_gymnasium.jbox2d.testbed.tests.Web;

/**
 * @author Daniel Murphy
 */
public class TestList
{
    public static void populateModel(TestbedModel model)
    {
        // particles
        model.addCategory("- Particles -");
        model.addTest(new BulletTest());
        model.addTest(new DamBreak());
        model.addTest(new DrawingParticles());
        model.addTest(new LiquidTimer());
        model.addTest(new WaveMachine());
        model.addTest(new Particles());
        model.addCategory("- Featured -");
        model.addTest(new DominoTest());
        model.addTest(new Car());
        model.addTest(new CompoundShapes());
        model.addTest(new BlobTest4());
        model.addTest(new TheoJansen());
        // watching...
        model.addCategory("- Collision Watching -");
        model.addTest(new VaryingRestitution());
        model.addTest(new VaryingFrictionTest());
        model.addTest(new ConveyorBelt());
        model.addTest(new SphereStack());
        model.addTest(new Tumbler());
        model.addTest(new PistonTest());
        model.addTest(new PyramidTest());
        model.addTest(new CircleStress());
        model.addTest(new DominoTower());
        // more interactive..
        model.addCategory("- Interactive -");
        model.addTest(new VerticalStack());
        model.addTest(new Breakable());
        model.addTest(new ShapeEditing());
        model.addTest(new OneSidedTest());
        model.addTest(new PolyShapes());
        model.addTest(new BodyTypes());
        model.addTest(new CharacterCollision());
        model.addTest(new ApplyForce());
        // processing/filtering
        model.addCategory("- Processing/Filtering -");
        model.addTest(new CollisionFiltering());
        model.addTest(new CollisionProcessing());
        model.addTest(new SensorTest());
        // joints
        model.addCategory("- Joints -");
        model.addTest(new PrismaticTest());
        model.addTest(new RevoluteTest());
        model.addTest(new FixedPendulumTest(true));
        model.addTest(new FreePendulumTest(true));
        model.addTest(new MotorTest());
        model.addTest(new Chain());
        model.addTest(new RopeTest());
        model.addTest(new Pulleys());
        model.addTest(new Gears());
        model.addTest(new Web());
        model.addTest(new Cantilever());
        model.addTest(new SliderCrankTest());
        // ccd
        model.addCategory("- CCD -");
        model.addTest(new ContinuousTest());
        model.addTest(new ConfinedTest());
        // raycast
        model.addCategory("- Raycast -");
        model.addTest(new RayCastTest());
        model.addTest(new EdgeShapes());
        // misc
        model.addCategory("- Misc -");
        model.addTest(new ConvexHull());
        model.addTest(new DynamicTreeTest());
        model.addTest(new DistanceTest());
    }
}
