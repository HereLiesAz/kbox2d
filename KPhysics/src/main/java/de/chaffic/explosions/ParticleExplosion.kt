package de.chaffic.explosions

import de.chaffic.dynamics.Body
import de.chaffic.dynamics.Body
import de.chaffic.dynamics.World
import de.chaffic.geometry.Circle
import de.chaffic.math.Mat2
import de.chaffic.math.Vec2
import java.time.Instant

private data class Particle(val body: Body, val creationTime: Long)

/**
 * Simulates an explosion by creating a number of small, fast-moving particles.
 *
 * This class creates a set of particle bodies and launches them outwards from an epicenter.
 * These particles can then collide with other objects in the world, transferring momentum
 * and simulating the effect of shrapnel.
 *
 * Note: This class does not implement the [Explosion] interface directly, as its mechanism
 * is different from the impulse-based explosions.
 *
 * Example of creating a particle explosion:
 * ```kotlin
 * // Create a particle explosion with 50 particles that live for 2 seconds
 * val particleExplosion = ParticleExplosion(Vec2(300.0, 300.0), 50, 2.0)
 *
 * // Create the particles in the world
 * particleExplosion.createParticles(size = 2.0, density = 10, radius = 10, world = world)
 *
 * // Apply an initial impulse to the particles
 * particleExplosion.applyBlastImpulse(100.0)
 *
 * // In your game loop, periodically remove expired particles
 * particleExplosion.removeExpiredParticles(world)
 * ```
 *
 * @property particles A list of the particle bodies created by the explosion.
 *
 * @param epicentre The center point from which particles will be spawned.
 * @param noOfParticles The total number of particles to create.
 * @param lifespan The life time of the particles in seconds.
 */
class ParticleExplosion(private val epicentre: Vec2, private val noOfParticles: Int, private val lifespan: Double) {
    private val particles = mutableListOf<Particle>()

    /**
     * Creates the particle bodies and adds them to the specified world.
     * The particles are arranged in a circle around the epicenter.
     *
     * @param size The radius of each particle.
     * @param density The density of each particle.
     * @param radius The radius of the circle on which particles are initially placed.
     * @param world The world to add the particles to.
     */
    fun createParticles(size: Double, density: Int, radius: Int, world: World) {
        val separationAngle = 6.28319 / noOfParticles
        val distanceFromCentre = Vec2(.0, radius.toDouble())
        val rotate = Mat2(separationAngle)
        for (i in 0 until noOfParticles) {
            val particlePlacement = epicentre.plus(distanceFromCentre)
            val b = Body(Circle(size), particlePlacement.x, particlePlacement.y)
            b.density = density.toDouble()
            b.restitution = 1.0
            b.staticFriction = 0.0
            b.dynamicFriction = 0.0
            b.affectedByGravity = false
            b.linearDampening = 0.0
            b.particle = true
            world.addBody(b)
            particles.add(Particle(b))
            rotate.mul(distanceFromCentre)
        }
    }

    /**
     * Applies an initial outward impulse to all particles, sending them flying from the epicenter.
     *
     * @param blastPower The magnitude of the impulse to apply to each particle.
     */
    fun applyBlastImpulse(blastPower: Double) {
        var line: Vec2
        for (p in particles) {
            line = p.body.position.minus(epicentre)
            p.body.velocity.set(line.scalar(blastPower))
        }
    }

    /**
     * Removes expired particles from the world.
     * This should be called periodically (e.g., in your game loop) to clean up old particles.
     *
     * @param world The world from which to remove the particles.
     */
    fun removeExpiredParticles(world: World) {
        val now = Instant.now().toEpochMilli()
        particles.removeIf { particle ->
            val isExpired = now - particle.creationTime > lifespan * 1000
            if (isExpired) {
                world.removeBody(particle.body)
            }
            isExpired
        }
    }
}