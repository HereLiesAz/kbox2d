package de.chaffic.explosions

import de.chaffic.dynamics.bodies.PhysicalBodyInterface
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2
import de.chaffic.rays.RayInformation

/**
 * Represents an explosion that affects bodies based on line-of-sight from the epicenter.
 *
 * This type of explosion casts a number of rays out from its epicenter. If a ray intersects
 * a body, an impulse is applied at the point of intersection. This simulates explosions
 * that are blocked by obstacles.
 *
 * Example of using a raycast explosion:
 * ```kotlin
 * // Create a raycast explosion with 36 rays, each 500 units long
 * val explosion = RaycastExplosion(
 *     epicentre = Vec2(300.0, 300.0),
 *     noOfRays = 36,
 *     distance = 500.0,
 *     worldBodies = world.bodies
 * )
 *
 * // Apply the blast impulse to the affected bodies
 * explosion.applyBlastImpulse(10000.0)
 * ```
 *
 * @property rayScatter The [RayScatter] instance used to cast the rays for the explosion.
 *
 * @param epicentre The center point of the explosion in world coordinates.
 * @param noOfRays The number of rays to cast out from the epicenter.
 * @param distance The maximum length of each ray.
 * @param worldBodies The list of bodies in the world to test for intersection.
 */
class RaycastExplosion(epicentre: Vec2, noOfRays: Int, distance: Double, worldBodies: ArrayList<TranslatableBody>) : Explosion {
    val rayScatter: RayScatter

    /**
     * Moves the epicenter of the explosion to a new position.
     *
     * @param v The new position for the explosion's epicenter.
     */
    override fun setEpicentre(v: Vec2) {
        rayScatter.epicentre = v
    }

    private var raysInContact = ArrayList<RayInformation>()

    init {
        rayScatter = RayScatter(epicentre, noOfRays)
        rayScatter.castRays(distance)
        update(worldBodies)
    }

    /**
     * Updates the explosion by recasting all rays and determining which bodies are hit.
     *
     * @param bodiesToEvaluate The list of bodies in the world to check for intersection.
     */
    override fun update(bodiesToEvaluate: ArrayList<TranslatableBody>) {
        raysInContact.clear()
        rayScatter.updateRays(bodiesToEvaluate)
        val rayArray = rayScatter.rays
        for (ray in rayArray) {
            val rayInfo = ray.rayInformation
            if (rayInfo != null) {
                raysInContact.add(rayInfo)
            }
        }
    }

    /**
     * Applies a blast impulse to each body hit by a ray. The impulse is applied at the point
     * of intersection and is directed away from the epicenter.
     *
     * @param blastPower The base magnitude of the blast impulse.
     */
    override fun applyBlastImpulse(blastPower: Double) {
        for (ray in raysInContact) {
            val blastDir = ray.coordinates.minus(rayScatter.epicentre)
            val distance = blastDir.length()
            if (distance == 0.0) return
            val invDistance = 1 / distance
            val impulseMag = blastDir.normalize().scalar(blastPower * invDistance)
            val b = ray.b
            if(b !is PhysicalBodyInterface) continue
            b.applyLinearImpulse(impulseMag, ray.coordinates.minus(b.position))
        }
    }
}