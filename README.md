[![Maven Central](https://img.shields.io/maven-central/v/de.pirckheimer-gymnasium/jbox2d-library.svg?style=flat)](https://central.sonatype.com/artifact/de.pirckheimer-gymnasium/jbox2d-library)
[![javadoc](https://javadoc.io/badge2/de.pirckheimer-gymnasium/jbox2d-library/javadoc.svg)](https://javadoc.io/doc/de.pirckheimer-gymnasium/jbox2d-library)

# kbox2d

Why this fork? Because not spoon. Maybe chopsticks. Maybe.

jbox2d has not been updated on Maven Central for a long time:
https://central.sonatype.com/artifact/org.jbox2d/jbox2d

And jbox2d hasn't worked as a Kotlin library in, like, FOREVER. 


JBox2d is a Java port of the C++ physics engines
[LiquidFun](http://google.github.io/liquidfun/) and [Box2d](http://box2d.org).

If you've downloaded this as an archive, you should find the built java jars in
the 'target' directories of each project.

---

- `kbox2d-library` - this is the main physics library. The only dependency is the
  SLF4J logging library.
- `kbox2d-serialization` - this adds serialization tools. Requires google's
  protocol buffer library installed to fully build
  (http://code.google.com/p/protobuf/), but this is optional, as the generated
  sources are included.
- `kbox2d-testbed` - A simple framework for creating and running physics tests.
- `kbox2d-testbed-jogl` - The testbed with OpenGL rendering.
- `kbox2d-jni-broadphase` - Experiment with moving parts of the engine to C++. Not
  faster.

## Tests / Demos

https://github.com/engine-pi/jbox2d/tree/main/jbox2d-testbed/src/main/java/de/pirckheimer_gymnasium/jbox2d/testbed/tests

<!-- ApplyForce
BlobTest4
BodyTypes
Breakable -->

BulletTest

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/BulletTest.gif)

<!-- Cantilever -->

Car

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/Car.gif)

<!-- Chain
CharacterCollision
CircleStress
CollisionFiltering
CollisionProcessing
CompoundShapes
ConfinedTest
ContinuousTest
ConvexHull
ConveyorBelt -->

DamBreak

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/DamBreak.gif)

<!-- DistanceTest -->

DominoTest

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/Dominos.gif)

<!--
DominoTower
DrawingParticles
DynamicTreeTest
EdgeShapes
EdgeTest
FixedPendulumTest
FreePendulumTest
Gears -->

LiquidTimer

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/LiquidTimer.gif)

<!--
MotorTest
OneSidedTest
Particles -->

ParticleTypes

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/Particles.gif)

<!-- PistonTest
PolyShapes
PrismaticTest
Pulleys
PyramidTest
RayCastTest
RevoluteTest
RopeTest
SensorTest
ShapeEditing
SliderCrankTest
SphereStack
TheoJansen
Tumbler
VaryingFrictionTest
VaryingRestitution
VerticalStack -->

WaveMachine

![](https://raw.githubusercontent.com/engine-pi/jbox2d/main/screencasts/WaveMachine.gif)

<!-- Web -->
