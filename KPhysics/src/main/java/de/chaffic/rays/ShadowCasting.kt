package de.chaffic.rays

import de.chaffic.collision.Arbiter.Companion.isPointInside
import de.chaffic.collision.bodies.CollisionBodyInterface
import de.chaffic.geometry.Circle
import de.chaffic.geometry.Polygon
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2
import kotlin.math.asin
import kotlin.math.atan2

/**
 * A utility for 2D shadow casting or field-of-view calculations.
 *
 * This class works by casting rays from a central point towards the vertices of all relevant
 * polygons and the tangent points of circles. By connecting the intersection points of these
 * rays, you can construct a polygon representing the visible area or the area not in shadow.
 *
 * Example of using ShadowCasting:
 * ```kotlin
 * // Create a shadow caster at the light source's position
 * val shadowCaster = ShadowCasting(lightSourcePosition, 1000.0)
 *
 * // Update it with the bodies that should cast shadows
 * shadowCaster.updateProjections(world.bodies)
 *
 * // The rayData now contains a sorted list of ray intersections
 * // which can be used to draw the light polygon.
 * val lightPolygonPoints = shadowCaster.rayData.map { it.ray.rayInformation?.coordinates ?: it.ray.endPoint }
 *
 * ```
 *
 * @property startPoint The origin point from which rays are cast (e.g., the light source position).
 * @property rayData A list of [RayAngleInformation] objects, containing all the cast rays and their intersection data, sorted by angle.
 *
 * @param startPoint The initial origin for projecting rays.
 * @param distance The maximum distance the rays will be projected.
 */
class ShadowCasting(var startPoint: Vec2, private val distance: Double) {

    val rayData = ArrayList<RayAngleInformation>()

    /**
     * Casts rays towards the key features (vertices, tangents) of the provided bodies
     * and updates the list of ray intersections. The results are sorted by angle and
     * stored in the [rayData] property.
     *
     * @param bodiesToEvaluate A list of bodies to cast shadows from.
     */
    fun updateProjections(bodiesToEvaluate: ArrayList<TranslatableBody>) {
        rayData.clear()
        for (B in bodiesToEvaluate) {
            if(B !is CollisionBodyInterface) continue

            if (isPointInside(B, startPoint)) {
                rayData.clear()
                break
            }
            if (B.shape is Polygon) {
                val poly1 = B.shape as Polygon
                for (v in poly1.vertices) {
                    val direction = poly1.orientation.mul(v, Vec2()).plus(B.position).minus(startPoint)
                    projectRays(direction, bodiesToEvaluate)
                }
            } else {
                val circle = B.shape as Circle
                val d = B.position.minus(startPoint)
                val angle = asin(circle.radius / d.length())
                val u = Mat2(angle)
                projectRays(u.mul(d.normalize(), Vec2()), bodiesToEvaluate)
                val u2 = Mat2(-angle)
                projectRays(u2.mul(d.normalize(), Vec2()), bodiesToEvaluate)
            }
        }
        rayData.sortWith { lhs: RayAngleInformation, rhs: RayAngleInformation ->
            rhs.angle.compareTo(lhs.angle)
        }
    }

    /**
     * Projects three slightly perturbed rays in a given direction and stores their intersection results.
     * Casting multiple rays helps to avoid missing corners due to floating point inaccuracies.
     *
     * @param direction The base direction in which to project the rays.
     * @param bodiesToEvaluate A list of bodies to check for intersection.
     */
    private fun projectRays(direction: Vec2, bodiesToEvaluate: ArrayList<TranslatableBody>) {
        val m = Mat2(0.001)
        m.transpose().mul(direction)
        for (i in 0..2) {
            val ray = Ray(startPoint, direction, distance)
            ray.updateProjection(bodiesToEvaluate)
            rayData.add(RayAngleInformation(ray, atan2(direction.y, direction.x)))
            m.mul(direction)
        }
    }

    /**
     * @return The total number of rays projected in the last update.
     */
    val noOfRays: Int
        get() = rayData.size
}

/**
 * A data class that holds a [Ray] and its angle.
 * This is used by [ShadowCasting] to sort ray intersections by angle.
 *
 * @property ray The ray object.
 * @property angle The angle of the ray in radians.
 *
 * @param ray The [Ray] object.
 * @param angle The angle of the ray, used for sorting.
 */
data class RayAngleInformation(val ray: Ray, val angle: Double)