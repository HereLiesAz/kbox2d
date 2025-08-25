# Getting Started with KPhysics

Welcome to KPhysics! This guide will walk you through the basics of using the KPhysics engine to create 2D physics simulations in your Kotlin or Java projects.

## 1. Introduction

KPhysics is a 2D physics engine designed for ease of use and performance. It provides a simple API for creating and managing rigid bodies, handling collisions, and simulating physical interactions like gravity, friction, and joints.

This guide will cover the essential concepts you need to get started. For a complete list of all available classes and functions, please refer to the [API Reference](index.html).

## 2. Setup

To use KPhysics in your project, you need to add it as a dependency using a build tool like Maven or Gradle. KPhysics is hosted on Jitpack, which makes it easy to include in your project.

### Maven

Add the Jitpack repository and the KPhysics dependency to your `pom.xml`:

```xml
<repositories>
    <repository>
        <id>jitpack.io</id>
        <url>https://jitpack.io</url>
    </repository>
</repositories>

<dependencies>
    <dependency>
        <groupId>com.github.Chafficui</groupId>
        <artifactId>KPhysics</artifactId>
        <version>Tag</version> <!-- Replace with the latest version tag -->
    </dependency>
</dependencies>
```

### Gradle

Add the Jitpack repository to your `build.gradle` or `build.gradle.kts` file:

```groovy
allprojects {
    repositories {
        maven { url 'https://jitpack.io' }
    }
}
```

Then, add the KPhysics dependency:

```groovy
dependencies {
    implementation 'com.github.Chafficui:KPhysics:Tag' // Replace with the latest version tag
}
```

## 3. Creating a World

The first step in any KPhysics simulation is to create a `World`. The `World` is the container for all your physics objects and is responsible for running the simulation.

You can create a world with a global gravity vector. If you don't specify one, the world will have no gravity.

```kotlin
import de.chaffic.dynamics.World
import de.chaffic.math.Vec2

// Create a world with a gravity of -9.81 on the y-axis
val world = World(Vec2(0.0, -9.81))
```

## 4. Creating Bodies

Bodies are the physical objects in your simulation. Each body has a shape, position, and various physical properties.

### Shapes

KPhysics provides two primary shapes: `Circle` and `Polygon`.

- `Circle`: A simple circular shape defined by a radius.
- `Polygon`: A convex polygon defined by a set of vertices. You can also create rectangles and regular polygons easily.

### Creating a Body

To create a body, you need to instantiate the `Body` class with a shape and an initial position.

```kotlin
import de.chaffic.dynamics.Body
import de.chaffic.geometry.Circle
import de.chaffic.geometry.Polygon

// Create a circular body (a ball)
val ballShape = Circle(20.0)
val ball = Body(ballShape, 100.0, 300.0)

// Create a rectangular body (the ground)
val groundShape = Polygon(800.0, 50.0)
val ground = Body(groundShape, 400.0, 50.0)
```

### Body Properties

You can customize the behavior of bodies by setting their properties.

- **Density**: Determines the mass of the body. A density of `0.0` makes the body **static**, meaning it will not move.
- **Restitution**: The "bounciness" of the body. A value of `0.0` means no bounce, while `1.0` means a perfectly elastic bounce.
- **Friction**: `staticFriction` and `dynamicFriction` control how the body interacts with other surfaces.

```kotlin
// Make the ground static and immovable
ground.density = 0.0

// Make the ball bouncy
ball.restitution = 0.8

// Set friction properties
ball.staticFriction = 0.5
ball.dynamicFriction = 0.3
```

### Adding Bodies to the World

Once you have created your bodies, you need to add them to the world to make them part of the simulation.

```kotlin
world.addBody(ball)
world.addBody(ground)
```

## 5. Running the Simulation

To run the simulation, you need to call the `world.step()` method in your game loop. This method advances the physics simulation by a given time step.

It's important to use a fixed time step for a stable and deterministic simulation.

```kotlin
fun gameLoop() {
    val deltaTime = 1.0 / 60.0 // 60 FPS

    // Update the physics world
    world.step(deltaTime)

    // Now, you can get the updated positions of your bodies and render them
    val ballPosition = ball.position
    // ... render the ball at its new position ...
}
```

## 6. Joints

Joints are used to connect bodies together. KPhysics provides two main types of joints:

- `JointToBody`: Connects two bodies.
- `JointToPoint`: Connects a body to a fixed point in the world.

Here's an example of creating a pendulum using a `JointToPoint`:

```kotlin
import de.chaffic.joints.JointToPoint

val anchor = Vec2(400.0, 500.0)
val pendulumBob = Body(Circle(25.0), 400.0, 300.0)

val joint = JointToPoint(
    pendulumBob,
    anchor,
    jointLength = 200.0,
    springConstant = 1000.0,
    dampening = 10.0,
    canGoSlack = false,
    offset = Vec2(0.0, 0.0)
)

world.addBody(pendulumBob)
world.addJoint(joint)
```

## 7. RayCasting

You can use the `Ray` class to cast rays into the world and find out what they hit. This is useful for things like line-of-sight checks or laser beams.

```kotlin
import de.chaffic.rays.Ray

// Create a ray starting at (0, 0) and pointing to (500, 500)
val ray = Ray(Vec2(0.0, 0.0), Vec2(1.0, 1.0), 707.0)

// Check for intersections with bodies in the world
ray.updateProjection(world.bodies)

if (ray.rayInformation != null) {
    println("Ray hit body: ${ray.rayInformation.b}")
    println("Intersection at: ${ray.rayInformation.coordinates}")
}
```

## 8. Explosions

KPhysics includes several types of explosions. Here's an example of a `ProximityExplosion`, which affects all bodies within a certain radius.

```kotlin
import de.chaffic.explosions.ProximityExplosion

// Create an explosion at (400, 300) with a radius of 200
val explosion = ProximityExplosion(Vec2(400.0, 300.0), 200)

// Find all bodies affected by the explosion
explosion.update(world.bodies)

// Apply the blast impulse
explosion.applyBlastImpulse(5000.0)
```

## 9. Next Steps

This guide has covered the basics of KPhysics. To learn more, you can explore the following resources:

- **[API Reference](index.html)**: The complete API documentation for all classes and functions.
- **Examples**: Check out the tech demos for more advanced examples of what KPhysics can do.

Happy coding!
