package de.chaffic.explosions

import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2

/**
 * Defines the common behavior for all explosion types in the physics engine.
 *
 * An explosion is an event that applies forces to bodies within its area of effect.
 * This interface provides a standard way to trigger and update explosions.
 *
 * Concrete implementations include:
 * - [ProximityExplosion]: Affects all bodies within a certain radius.
 * - [RaycastExplosion]: Affects bodies that have a clear line of sight to the explosion's epicenter.
 * - [ParticleExplosion]: Simulates a shower of particles, each of which can interact with bodies.
 *
 * @see ProximityExplosion
 * @see RaycastExplosion
 * @see ParticleExplosion
 */
interface Explosion {
    /**
     * Applies a blast impulse to all bodies affected by the explosion.
     * The magnitude and direction of the impulse depend on the explosion type and the body's position.
     *
     * @param blastPower The base magnitude of the impulse to be applied.
     */
    fun applyBlastImpulse(blastPower: Double)

    /**
     * Updates the state of the explosion, typically by re-evaluating which bodies are affected.
     * This is useful for explosions that are not instantaneous.
     *
     * @param bodiesToEvaluate A list of all bodies in the world to check for interaction with the explosion.
     */
    fun update(bodiesToEvaluate: ArrayList<TranslatableBody>)

    /**
     * Moves the epicenter of the explosion to a new position.
     *
     * @param v The new position for the explosion's epicenter in world coordinates.
     */
    fun setEpicentre(v: Vec2)
}