package de.chaffic.explosions

import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2
import de.chaffic.rays.Ray

/**
 * A utility class that creates and manages a collection of rays originating from a single point,
 * spreading out in a 360-degree pattern.
 *
 * This class is used by [RaycastExplosion] to simulate the effects of an explosion that is
 * blocked by obstacles. It can also be used for other purposes, such as AI line-of-sight checks.
 *
 * Example of using RayScatter:
 * ```kotlin
 * // Create a ray scatter at (100, 100) with 12 rays
 * val rayScatter = RayScatter(Vec2(100.0, 100.0), 12)
 *
 * // Cast the rays with a length of 200
 * rayScatter.castRays(200.0)
 *
 * // Update the rays to find intersections with bodies in the world
 * rayScatter.updateRays(world.bodies)
 *
 * // Now you can inspect the individual rays for intersection information
 * for (ray in rayScatter.rays) {
 *     if (ray.rayInformation != null) {
 *         println("Ray hit a body at: ${ray.rayInformation.coordinates}")
 *     }
 * }
 * ```
 *
 * @property rays A list of the [Ray] objects managed by this scatter.
 * @property epicentre The origin point from which all rays are cast. Setting this will update the start point of all rays.
 *
 * @param epicentre The initial origin point for the rays.
 * @param noOfRays The number of rays to create in the scatter pattern.
 */
class RayScatter(epicentre: Vec2, private val noOfRays: Int) {
    val rays = mutableListOf<Ray>()
    var epicentre: Vec2 = epicentre
        set(value) {
            field = value
            for (ray in rays) {
                ray.startPoint = field
            }
        }

    /**
     * Creates the rays in a 360-degree pattern with equal angular spacing.
     *
     * @param distance The length of each ray.
     */
    fun castRays(distance: Double) {
        val angle = 6.28319 / noOfRays
        val direction = Vec2(1.0, 1.0)
        val u = Mat2(angle)
        for (i in rays.indices) {
            rays.add(Ray(epicentre, direction, distance))
            u.mul(direction)
        }
    }

    /**
     * Updates all rays in the scatter, checking for intersections with the provided list of bodies.
     *
     * @param worldBodies A list of bodies to check for intersection against.
     */
    fun updateRays(worldBodies: ArrayList<TranslatableBody>) {
        for (ray in rays) {
            ray.updateProjection(worldBodies)
        }
    }
}