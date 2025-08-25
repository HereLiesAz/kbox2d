package de.chaffic.geometry

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.dynamics.bodies.PhysicalBodyInterface
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2
import kotlin.math.sqrt

/**
 * Represents a circle shape to be used in a physics body.
 *
 * A circle is defined by its radius. It is a simple and efficient shape for collision detection.
 *
 * Example of creating a circular body:
 * ```kotlin
 * // Create a circle shape with a radius of 25.0
 * val circleShape = Circle(25.0)
 *
 * // Create a body with this shape at position (100, 100)
 * val circleBody = Body(circleShape, 100.0, 100.0)
 *
 * // Add the body to the world
 * world.addBody(circleBody)
 * ```
 *
 * @property radius The radius of the circle.
 * @param radius The desired radius of the circle.
 *
 * @see Shape
 * @see Body
 */
class Circle(var radius: Double) : Shape() {
    /**
     * Calculates the mass and inertia of the circle based on its area and the given density.
     * The results are stored in the parent [Body].
     *
     * @param density The density of the material, used to calculate mass.
     */
    override fun calcMass(density: Double) {
        val physicalBody = this.body
        if(physicalBody !is PhysicalBodyInterface) return
        physicalBody.mass = StrictMath.PI * radius * radius * density
        physicalBody.invMass = if (physicalBody.mass != 0.0) 1.0f / physicalBody.mass else 0.0
        physicalBody.inertia = physicalBody.mass * radius * radius
        physicalBody.invInertia = if (physicalBody.inertia != 0.0) 1.0f / physicalBody.inertia else 0.0
    }

    /**
     * Generates an axis-aligned bounding box (AABB) for the circle.
     * The AABB is centered around the circle's position and is large enough to fully contain it.
     */
    override fun createAABB() {
        this.body.aabb = AxisAlignedBoundingBox(Vec2(-radius, -radius), Vec2(radius, radius))
    }

    /**
     * Checks if a given point is inside the circle.
     *
     * @param startPoint The point to check, in world coordinates.
     * @return `true` if the point is inside or on the boundary of the circle, `false` otherwise.
     */
    override fun isPointInside(startPoint: Vec2): Boolean {
        val d = this.body.position.minus(startPoint)
        return d.length() <= radius
    }

    /**
     * Performs a ray-intersection test with the circle.
     *
     * @param startPoint The starting point of the ray in world coordinates.
     * @param endPoint The end point of the ray in world coordinates.
     * @param maxDistance The maximum distance for a valid intersection.
     * @param rayLength The total length of the ray.
     * @return An [IntersectionReturnElement] containing information about the intersection, if one occurred.
     */
    override fun rayIntersect(startPoint: Vec2, endPoint: Vec2, maxDistance: Double, rayLength: Double): IntersectionReturnElement {
        var minPx = 0.0
        var minPy = 0.0
        var intersectionFound = false
        var closestBody: TranslatableBody? = null
        var maxD = maxDistance

        val ray = endPoint.copy().minus(startPoint)
        val circleCenter = body.position.copy()
        val r = radius
        val difInCenters = startPoint.minus(circleCenter)
        val a = ray.dot(ray)
        val b = 2 * difInCenters.dot(ray)
        val c = difInCenters.dot(difInCenters) - r * r
        var discriminant = b * b - 4 * a * c
        if (discriminant >= 0) {
            discriminant = sqrt(discriminant)
            val t1 = (-b - discriminant) / (2 * a)
            if (t1 in 0.0..1.0) {
                if (t1 < maxDistance) {
                    maxD = t1
                    minPx = startPoint.x + endPoint.x * t1
                    minPy = startPoint.y + endPoint.y * t1
                    intersectionFound = true
                    closestBody = body
                }
            }
        }
        return IntersectionReturnElement(minPx, minPy, intersectionFound, closestBody, maxD)
    }
}