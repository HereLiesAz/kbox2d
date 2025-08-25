package de.chaffic.rays

import de.chaffic.dynamics.Body
import de.chaffic.dynamics.World
import de.chaffic.geometry.Circle
import de.chaffic.geometry.Polygon
import de.chaffic.math.Vec2
import junit.framework.TestCase
import org.junit.Test

class RayTest : TestCase() {

    @Test
    fun testPolygonRight() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), 300.0, 0.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.RIGHT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(250.0, .0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonLeft() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), -300.0, 0.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.LEFT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(-250.0, .0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonUp() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), 0.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.UP, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(.0, 200.0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonDown() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), 0.0, -300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.DOWN, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(.0, -200.0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonRightOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), 300.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, 300.0), Vec2.RIGHT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(250.0, 300.0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonLeftOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), -300.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, 300.0), Vec2.LEFT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(-250.0, 300.0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonUpOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), 300.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(300.0, .0), Vec2.UP, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(300.0, 200.0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonDownOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), 300.0, -300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(300.0, .0), Vec2.DOWN, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
        assertEquals(Vec2(300.0, -200.0), ray.rayInformation?.coordinates)
    }

    @Test
    fun testPolygonDiagonal() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Polygon(50.0, 100.0, true), .0, -300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(300.0, .0), Vec2(1.0, 1.0), 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(null, ray.rayInformation)
    }

    @Test
    fun testCircleRight() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), 300.0, 0.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.RIGHT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleLeft() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), -300.0, 0.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.LEFT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleUp() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), 0.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.UP, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleDown() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), 0.0, -300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, .0), Vec2.DOWN, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleRightOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), 300.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, 300.0), Vec2.RIGHT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleLeftOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), -300.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(.0, 300.0), Vec2.LEFT, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleUpOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), 300.0, 300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(300.0, .0), Vec2.UP, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }

    @Test
    fun testCircleDownOffset() {
        val world = World(Vec2(.0, -9.81))

        val plattform = Body(Circle(50.0), 300.0, -300.0)
        plattform.density = .0

        world.addBody(plattform)

        val ray = Ray(Vec2(300.0, .0), Vec2.DOWN, 800.0)
        ray.updateProjection(world.bodies)

        assertEquals(plattform, ray.rayInformation?.b)
    }
}