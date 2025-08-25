package de.chaffic.dynamics

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.collision.bodies.CollisionBodyInterface
import de.chaffic.dynamics.bodies.AbstractPhysicalBody
import de.chaffic.geometry.Shape
import de.chaffic.math.Vec2

/**
 * A `Body` is a fundamental object in the physics world, representing a physical entity with properties
 * like position, mass, friction, and restitution. Bodies are affected by forces and collisions.
 *
 * Each body must be initialized with a [Shape] (e.g., [de.chaffic.geometry.Circle] or [de.chaffic.geometry.Polygon]),
 * which defines its geometry, and an initial position in the world.
 *
 * Example of creating a body and setting its properties:
 * ```kotlin
 * // Create a polygonal body at position (100, 50)
 * val ground = Body(Polygon(800.0, 50.0), 100.0, 50.0)
 *
 * // Set its density to 0 to make it a static, immovable object
 * ground.setDensity(0.0)
 *
 * // Create a dynamic circular body
 * val ball = Body(Circle(20.0), 100.0, 200.0)
 *
 * // Set its restitution to make it bouncy
 * ball.restitution = 0.8
 *
 * // Add the bodies to the world to make them active
 * world.addBody(ground)
 * world.addBody(ball)
 * ```
 *
 * @property shape The geometric shape of the body, which determines its collision profile.
 * @property position The position of the body's center of mass in world coordinates.
 * @property dynamicFriction The coefficient of dynamic friction, used when the body is moving.
 * @property staticFriction The coefficient of static friction, used when the body is at rest.
 * @property orientation The orientation of the body in radians.
 * @property aabb The axis-aligned bounding box of the body, used for broad-phase collision detection.
 *
 * @param shape The [Shape] to bind to this body.
 * @param x The initial x-coordinate of the body's position in world space.
 * @param y The initial y-coordinate of the body's position in world space.
 *
 * @see World
 * @see Shape
 */
class Body(override var shape: Shape, x: Double, y: Double): AbstractPhysicalBody(), CollisionBodyInterface {
    override var position: Vec2 = Vec2(x, y)
    override var dynamicFriction = .2
    override var staticFriction = .5
    override var orientation = .0
        set(value) {
            field = value
            shape.orientation.set(orientation)
            shape.createAABB()
        }
    override lateinit var aabb: AxisAlignedBoundingBox

    init {
        density = density
        shape.body = this
        shape.orientation.set(orientation)
        shape.createAABB()
    }
}