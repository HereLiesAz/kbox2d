package de.chaffic.rays

import de.chaffic.collision.bodies.CollisionBodyInterface
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2

/**
 * Represents a ray that can be cast into the world to find intersecting bodies.
 *
 * A ray is defined by a starting point, a direction vector, and a maximum distance.
 * It can be used for line-of-sight checks, picking objects, or creating effects like lasers.
 *
 * After casting the ray using [updateProjection], the [rayInformation] property will contain
 * details about the closest intersection, if any.
 *
 * Example of casting a ray:
 * ```kotlin
 * // Create a ray starting at (0,0), pointing down and to the right, with a length of 500
 * val ray = Ray(Vec2(0.0, 0.0), Vec2(1.0, -1.0), 500.0)
 *
 * // Cast the ray against the bodies in the world
 * ray.updateProjection(world.bodies)
 *
 * // Check if the ray hit something
 * if (ray.rayInformation != null) {
 *     println("Ray hit body ${ray.rayInformation.b} at ${ray.rayInformation.coordinates}")
 * }
 * ```
 *
 * @property startPoint The starting point of the ray in world coordinates.
 * @property distance The maximum length of the ray.
 * @property direction The normalized direction vector of the ray.
 * @property rayInformation Contains information about the closest intersection found by the ray, or `null` if no intersection occurred.
 *
 * @param startPoint The origin of the ray's projection.
 * @param direction The direction vector of the ray (will be normalized).
 * @param distance The maximum distance the ray will travel.
 */
class Ray(var startPoint: Vec2, direction: Vec2, distance: Double) {
    val distance: Double
    var direction: Vec2

    /**
     * Creates a ray starting at the origin (0,0).
     *
     * @param direction The angle of the ray in radians.
     * @param distance The maximum distance the ray will travel.
     */
    constructor(direction: Double, distance: Double) : this(Vec2(), Vec2(direction), distance)

    /**
     * Creates a ray starting at the origin (0,0).
     *
     * @param direction The direction vector of the ray.
     * @param distance The maximum distance the ray will travel.
     */
    constructor(direction: Vec2, distance: Double) : this(Vec2(), direction, distance)

    /**
     * Creates a ray with a direction specified by an angle.
     *
     * @param startPoint The origin of the ray's projection.
     * @param direction The angle of the ray in radians.
     * @param distance The maximum distance the ray will travel.
     */
    constructor(startPoint: Vec2, direction: Double, distance: Double) : this(
        startPoint,
        Vec2(direction),
        distance
    )

    var rayInformation: RayInformation? = null
        private set

    init {
        this.direction = direction.normalized
        this.distance = distance
    }

    /**
     * Casts the ray into the world and finds the closest intersection with the given bodies.
     * The result is stored in the [rayInformation] property. If an intersection is found,
     * [rayInformation] will be a [RayInformation] object; otherwise, it will be `null`.
     *
     * @param bodiesToEvaluate A list of bodies to check for intersection.
     */
    fun updateProjection(bodiesToEvaluate: ArrayList<TranslatableBody>) {
        rayInformation = null
        val endPoint = direction.scalar(distance).plus(startPoint)
        var minT1 = Double.POSITIVE_INFINITY
        var minPx = 0.0
        var minPy = 0.0
        var intersectionFound = false
        var closestBody: TranslatableBody? = null
        for (body in bodiesToEvaluate) {
            if(body !is CollisionBodyInterface) continue
            val shape = body.shape
            val intersectionReturnElement = shape.rayIntersect(startPoint, endPoint, minT1, distance)
            if(intersectionReturnElement.intersectionFound) {
                minT1 = intersectionReturnElement.maxDistance
                minPx = intersectionReturnElement.minPx
                minPy = intersectionReturnElement.minPy
                intersectionFound = true
                closestBody = intersectionReturnElement.closestBody
            }
        }
        if (intersectionFound) {
            rayInformation = closestBody?.let { RayInformation(it, minPx, minPy, -1) }
        }
    }
}