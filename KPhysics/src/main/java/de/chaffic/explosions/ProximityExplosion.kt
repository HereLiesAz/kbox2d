package de.chaffic.explosions

import de.chaffic.dynamics.bodies.PhysicalBodyInterface
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2

/**
 * Represents an explosion that affects all bodies within a certain radius of its epicenter.
 *
 * The force of the explosion decreases with distance from the epicenter.
 *
 * Example of using a proximity explosion:
 * ```kotlin
 * // Create an explosion at (300, 300) with a radius of 200
 * val explosion = ProximityExplosion(Vec2(300.0, 300.0), 200)
 *
 * // Find which bodies are affected
 * explosion.update(world.bodies)
 *
 * // Apply the blast impulse
 * explosion.applyBlastImpulse(5000.0)
 * ```
 *
 * @property proximity The radius of the explosion's area of effect.
 * @property linesToBodies A list of vectors pointing from the epicenter to each affected body. Useful for debugging.
 *
 * @param epicentre The center point of the explosion in world coordinates.
 * @param proximity The radius within which bodies will be affected.
 */
class ProximityExplosion(private var epicentre: Vec2, val proximity: Int) : Explosion {
    /**
     * Moves the epicenter of the explosion to a new position.
     *
     * @param v The new position for the explosion's epicenter.
     */
    override fun setEpicentre(v: Vec2) {
        epicentre = v
    }

    /**
     * @return The current position of the explosion's epicenter.
     */
    fun getEpicentre(): Vec2 {
        return epicentre
    }

    private var bodiesEffected = ArrayList<TranslatableBody>()

    /**
     * Identifies all bodies within the explosion's proximity and adds them to the list of affected bodies.
     *
     * @param bodiesToEvaluate A list of all bodies in the world to check.
     */
    override fun update(bodiesToEvaluate: ArrayList<TranslatableBody>) {
        bodiesEffected.clear()
        for (b in bodiesToEvaluate) {
            val blastDist = b.position.minus(epicentre)
            if (blastDist.length() <= proximity) {
                bodiesEffected.add(b)
            }
        }
    }

    val linesToBodies = ArrayList<Vec2?>()

    /**
     * Updates the list of lines pointing to affected bodies. This is primarily for debugging and visualization purposes.
     */
    fun updateLinesToBody() {
        linesToBodies.clear()
        for (b in bodiesEffected) {
            linesToBodies.add(b.position)
        }
    }

    /**
     * Applies a blast impulse to all affected bodies. The impulse is directed away from the
     * epicenter and its magnitude is inversely proportional to the distance from the epicenter.
     *
     * @param blastPower The base magnitude of the blast impulse.
     */
    override fun applyBlastImpulse(blastPower: Double) {
        for (b in bodiesEffected) {
            if(b !is PhysicalBodyInterface) continue

            val blastDir = b.position.minus(epicentre)
            val distance = blastDir.length()
            if (distance == 0.0) return

            //Not physically correct as it should be blast * radius to object ^ 2 as the pressure of an explosion in 2D dissipates
            val invDistance = 1 / distance
            val impulseMag = blastPower * invDistance
            b.applyLinearImpulse(blastDir.normalize().scalar(impulseMag))
        }
    }
}