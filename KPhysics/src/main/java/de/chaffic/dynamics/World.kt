package de.chaffic.dynamics

import de.chaffic.collision.Arbiter
import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.collision.bodies.CollisionBodyInterface
import de.chaffic.dynamics.bodies.PhysicalBodyInterface
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.joints.Joint
import de.chaffic.math.Vec2
import kotlin.math.pow

/**
 * The `World` class is the main container for all physics objects and simulations.
 * It manages the bodies, joints, and the overall physics simulation steps.
 *
 * A world can be configured with a global gravity vector. If no gravity is specified,
 * it defaults to a zero vector, meaning no global gravity force will be applied to bodies.
 *
 * Example of creating a world, adding a body, and running the simulation:
 * ```kotlin
 * // Create a world with downward gravity
 * val world = World(Vec2(0.0, -9.81))
 *
 * // Create a circular body and add it to the world
 * val ball = Body(Circle(20.0), 0.0, 200.0)
 * world.addBody(ball)
 *
 * // In your game loop, update the world with the time delta
 * fun gameLoop(deltaTime: Double) {
 *     world.step(deltaTime)
 *     // ... render bodies ...
 * }
 * ```
 *
 * @property gravity The global gravity vector applied to all bodies in the world.
 * @property bodies A list of all bodies currently in the world.
 * @property joints A list of all joints currently in the world.
 * @property contacts A list of all contact arbiters generated during the collision phase.
 * @param gravity The strength of gravity in the world.
 */
class World(var gravity: Vec2 = Vec2()) {

    var bodies = ArrayList<TranslatableBody>()

    /**
     * Adds a body to the world, making it part of the physics simulation.
     *
     * @param body The [TranslatableBody] to add to the world.
     * @return The body that was added, with its concrete type preserved.
     * @see removeBody
     */
    fun <T : TranslatableBody> addBody(body: T): T {
        bodies.add(body)
        return body
    }

    /**
     * Removes a body from the world.
     *
     * @param b The body to remove from the world.
     * @see addBody
     */
    fun removeBody(b: TranslatableBody) {
        bodies.remove(b)
    }

    @JvmField
    var joints = ArrayList<Joint>()

    /**
     * Adds a joint to the world, which creates a constraint between bodies.
     *
     * @param j The joint to add.
     * @return The joint that was added.
     * @see removeJoint
     */
    fun addJoint(j: Joint): Joint {
        joints.add(j)
        return j
    }

    /**
     * Removes a joint from the world.
     *
     * @param j The joint to remove from the world.
     * @see addJoint
     */
    fun removeJoint(j: Joint) {
        joints.remove(j)
    }

    var contacts = ArrayList<Arbiter>()

    /**
     * Advances the physics simulation by a given time step.
     * This method performs collision detection, solves constraints, and updates the positions of all bodies.
     * It should be called once per frame in your game loop.
     *
     * @param dt The time step, in seconds, to advance the simulation by.
     */
    fun step(dt: Double) {
        contacts.clear()
        broadPhaseCheck()
        semiImplicit(dt)

        //Correct positional errors from the discrete collisions
        for (contact in contacts) {
            contact.penetrationResolution()
        }
    }

    /**
     * Semi implicit euler integration method for the world bodies and forces.
     *
     * @param dt Timestep
     */
    private fun semiImplicit(dt: Double) {
        //Applies tentative velocities
        applyForces(dt)
        solve()

        //Integrate positions
        for (b in bodies) {
            if(b !is PhysicalBodyInterface) continue
            if (b.invMass == 0.0) {
                continue
            }
            b.position.add(b.velocity.scalar(dt))
            if(b is CollisionBodyInterface) {
                b.orientation = b.orientation + dt * b.angularVelocity
            }
            b.force[0.0] = 0.0
            b.torque = 0.0
        }
    }

    /**
     * Applies semi-implicit euler and drag forces.
     *
     * @param dt Timestep
     */
    private fun applyForces(dt: Double) {
        for (b in bodies) {
            if(b !is PhysicalBodyInterface) continue
            if (b.invMass == 0.0) {
                continue
            }
            applyLinearDrag(b)
            if (b.affectedByGravity) {
                b.velocity.add(gravity.scalar(dt))
            }
            b.velocity.add(b.force.scalar(b.invMass).scalar(dt))
            b.angularVelocity += dt * b.invInertia * b.torque
        }
    }

    /**
     * Method to apply all forces in the world.
     */
    private fun solve() {
        /*
        Resolve joints
        Note: this is removed from the iterations at this stage as the application of forces is different.
        The extra iterations on joints make the forces of the joints multiple times larger equal to the number of iterations.
        Early out could be used like in the collision solver
        This may change in the future and will be revised at a later date.
        */
        for (j in joints) {
            j.applyTension()
        }

        //Resolve collisions
        for (i in 0 until Physics.ITERATIONS) {
            for (contact in contacts) {
                contact.solve()
            }
        }
    }

    /**
     * Applies linear drag to a body.
     *
     * @param b Body to apply drag to.
     */
    private fun applyLinearDrag(b: PhysicalBodyInterface?) {
        val velocityMagnitude = b!!.velocity.length()
        val dragForceMagnitude = velocityMagnitude * velocityMagnitude * b.linearDampening
        val dragForceVector = b.velocity.normalized.scalar(-dragForceMagnitude)
        b.applyForce(dragForceVector)
    }

    /**
     * A discrete Broad phase check of collision detection.
     */
    private fun broadPhaseCheck() {
        for (i in bodies.indices) {
            val a = bodies[i]
            for (x in i + 1 until bodies.size) {
                val b = bodies[x]
                if(a !is CollisionBodyInterface || b !is CollisionBodyInterface) continue

                //Ignores static or particle objects
                if (a is PhysicalBodyInterface && b is PhysicalBodyInterface && (a.invMass == 0.0 && b.invMass == 0.0 || a.particle && b.particle)) {
                    continue
                }
                if (AxisAlignedBoundingBox.aabbOverlap(a, b)) {
                    narrowPhaseCheck(a, b)
                }
            }
        }
    }

    /**
     * If broad phase detection check passes, a narrow phase check is conducted to determine for certain if two objects are intersecting.
     * If two objects are, arbiters of contacts found are generated
     *
     * @param a
     * @param b
     */
    private fun narrowPhaseCheck(a: CollisionBodyInterface, b: CollisionBodyInterface) {
        val contactQuery = Arbiter(a, b)
        contactQuery.narrowPhase()
        if (contactQuery.contactCount > 0) {
            contacts.add(contactQuery)
        }
    }

    /**
     * Removes all bodies, joints, and contacts from the world.
     * This is useful for resetting the simulation state.
     */
    fun clearWorld() {
        bodies.clear()
        contacts.clear()
        joints.clear()
    }

    /**
     * Applies gravitational forces between all pairs of objects in the world.
     * This method calculates the gravitational force based on the mass of the objects and the distance between them.
     * Note that this is different from the global `gravity` vector, which applies a constant force to all objects.
     * This method simulates n-body gravitational attraction.
     */
    fun gravityBetweenObj() {
        for (a in bodies.indices) {
            val bodyA = bodies[a]
            for (b in a + 1 until bodies.size) {
                val bodyB = bodies[b]
                if(bodyB !is PhysicalBodyInterface || bodyA !is PhysicalBodyInterface) continue
                val distance = bodyA.position.distance(bodyB.position)
                val force = 6.67.pow(-11.0) * bodyA.mass * bodyB.mass / (distance * distance)
                var direction: Vec2? = Vec2(bodyB.position.x - bodyA.position.x, bodyB.position.y - bodyA.position.y)
                direction = direction!!.scalar(force)
                val oppositeDir = Vec2(-direction.x, -direction.y)
                bodyA.force.plus(direction)
                bodyB.force.plus(oppositeDir)
            }
        }
    }
}