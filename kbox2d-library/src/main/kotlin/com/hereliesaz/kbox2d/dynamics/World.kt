package com.hereliesaz.kbox2d.dynamics

import com.hereliesaz.kbox2d.callbacks.ContactFilter
import com.hereliesaz.kbox2d.callbacks.ContactListener
import com.hereliesaz.kbox2d.callbacks.DebugDraw
import com.hereliesaz.kbox2d.callbacks.DestructionListener
import com.hereliesaz.kbox2d.callbacks.ParticleDestructionListener
import com.hereliesaz.kbox2d.callbacks.ParticleQueryCallback
import com.hereliesaz.kbox2d.callbacks.ParticleRaycastCallback
import com.hereliesaz.kbox2d.callbacks.QueryCallback
import com.hereliesaz.kbox2d.callbacks.RayCastCallback
import com.hereliesaz.kbox2d.callbacks.TreeCallback
import com.hereliesaz.kbox2d.callbacks.TreeRayCastCallback
import com.hereliesaz.kbox2d.collision.AABB
import com.hereliesaz.kbox2d.collision.RayCastInput
import com.hereliesaz.kbox2d.collision.RayCastOutput
import com.hereliesaz.kbox2d.collision.TimeOfImpact.TOIInput
import com.hereliesaz.kbox2d.collision.TimeOfImpact.TOIOutput
import com.hereliesaz.kbox2d.collision.TimeOfImpact.TOIOutputState
import com.hereliesaz.kbox2d.collision.broadphase.BroadPhase
import com.hereliesaz.kbox2d.collision.broadphase.BroadPhaseStrategy
import com.hereliesaz.kbox2d.collision.broadphase.DefaultBroadPhaseBuffer
import com.hereliesaz.kbox2d.collision.broadphase.DynamicTree
import com.hereliesaz.kbox2d.collision.shapes.ChainShape
import com.hereliesaz.kbox2d.collision.shapes.CircleShape
import com.hereliesaz.kbox2d.collision.shapes.EdgeShape
import com.hereliesaz.kbox2d.collision.shapes.PolygonShape
import com.hereliesaz.kbox2d.collision.shapes.Shape
import com.hereliesaz.kbox2d.collision.shapes.ShapeType
import com.hereliesaz.kbox2d.common.Color3f
import com.hereliesaz.kbox2d.common.MathUtils
import com.hereliesaz.kbox2d.common.Settings
import com.hereliesaz.kbox2d.common.Sweep
import com.hereliesaz.kbox2d.common.Timer
import com.hereliesaz.kbox2d.common.Transform
import com.hereliesaz.kbox2d.common.Vec2
import com.hereliesaz.kbox2d.dynamics.contacts.Contact
import com.hereliesaz.kbox2d.dynamics.contacts.ContactEdge
import com.hereliesaz.kbox2d.dynamics.contacts.ContactRegister
import com.hereliesaz.kbox2d.dynamics.joints.Joint
import com.hereliesaz.kbox2d.dynamics.joints.JointDef
import com.hereliesaz.kbox2d.dynamics.joints.JointEdge
import com.hereliesaz.kbox2d.dynamics.joints.PulleyJoint
import com.hereliesaz.kbox2d.particle.ParticleBodyContact
import com.hereliesaz.kbox2d.particle.ParticleColor
import com.hereliesaz.kbox2d.particle.ParticleContact
import com.hereliesaz.kbox2d.particle.ParticleDef
import com.hereliesaz.kbox2d.particle.ParticleGroup
import com.hereliesaz.kbox2d.particle.ParticleGroupDef
import com.hereliesaz.kbox2d.particle.ParticleSystem
import com.hereliesaz.kbox2d.pooling.IDynamicStack
import com.hereliesaz.kbox2d.pooling.IWorldPool
import com.hereliesaz.kbox2d.pooling.arrays.Vec2Array
import com.hereliesaz.kbox2d.pooling.normal.DefaultWorldPool

/**
 * The world class manages all physics entities, dynamic simulation, and asynchronous queries. The
 * world also contains efficient memory management facilities.
 *
 * @author Daniel Murphy
 */
class World @JvmOverloads constructor(
    gravity: Vec2,
    val pool: IWorldPool,
    broadPhase: BroadPhase = DefaultBroadPhaseBuffer(DynamicTree())
) {
    // statistics gathering
    var activeContacts = 0
    var contactPoolCount = 0
    protected var m_flags: Int
    protected var m_contactManager: ContactManager
    private var m_bodyList: Body? = null
    private var m_jointList: Joint? = null
    private var m_bodyCount = 0
    private var m_jointCount = 0
    private val m_gravity = Vec2()
    var isSleepingAllowed: Boolean
        private set

    // private Body m_groundBody;
    var destructionListener: DestructionListener? = null
        private set
    var particleDestructionListener: ParticleDestructionListener? = null
    var debugDraw: DebugDraw? = null
        private set

    /**
     * This is used to compute the time step ratio to support a variable time step.
     */
    private var m_inv_dt0 = 0f

    // these are for debugging the solver
    var isWarmStarting: Boolean
        private set
    var isContinuousPhysics: Boolean
        private set
    var isSubStepping = false
        private set
    private var m_stepComplete: Boolean
    private val m_profile: Profile
    private val m_particleSystem: ParticleSystem
    private val contactStacks =
        Array(ShapeType.values().size) { arrayOfNulls<ContactRegister>(ShapeType.values().size) }

    /**
     * Construct a world object.
     *
     * @param gravity the world gravity vector.
     */
    constructor(gravity: Vec2) : this(
        gravity,
        DefaultWorldPool(WORLD_POOL_SIZE, WORLD_POOL_CONTAINER_SIZE)
    ) {
    }

    /**
     * Construct a world object.
     *
     * @param gravity the world gravity vector.
     */
    constructor(gravity: Vec2, pool: IWorldPool, strategy: BroadPhaseStrategy) : this(
        gravity,
        pool,
        DefaultBroadPhaseBuffer(strategy)
    ) {
    }

    fun setAllowSleep(flag: Boolean) {
        if (flag == isSleepingAllowed) {
            return
        }
        isSleepingAllowed = flag
        if (isSleepingAllowed == false) {
            var b = m_bodyList
            while (b != null) {
                b.isAwake = true
                b = b.m_next
            }
        }
    }

    private fun addType(creator: IDynamicStack<Contact>, type1: ShapeType, type2: ShapeType) {
        val register = ContactRegister()
        register.creator = creator
        register.primary = true
        contactStacks[type1.ordinal][type2.ordinal] = register
        if (type1 != type2) {
            val register2 = ContactRegister()
            register2.creator = creator
            register2.primary = false
            contactStacks[type2.ordinal][type1.ordinal] = register2
        }
    }

    private fun initializeRegisters() {
        addType(pool.circleContactStack, ShapeType.CIRCLE, ShapeType.CIRCLE)
        addType(pool.polyCircleContactStack, ShapeType.POLYGON, ShapeType.CIRCLE)
        addType(pool.polyContactStack, ShapeType.POLYGON, ShapeType.POLYGON)
        addType(pool.edgeCircleContactStack, ShapeType.EDGE, ShapeType.CIRCLE)
        addType(pool.edgePolyContactStack, ShapeType.EDGE, ShapeType.POLYGON)
        addType(pool.chainCircleContactStack, ShapeType.CHAIN, ShapeType.CIRCLE)
        addType(pool.chainPolyContactStack, ShapeType.CHAIN, ShapeType.POLYGON)
    }

    fun popContact(fixtureA: Fixture, indexA: Int, fixtureB: Fixture, indexB: Int): Contact? {
        val type1 = fixtureA.type
        val type2 = fixtureB.type
        val reg = contactStacks[type1.ordinal][type2.ordinal]
        if (reg != null) {
            return if (reg.primary) {
                val c = reg.creator.pop()
                c.init(fixtureA, indexA, fixtureB, indexB)
                c
            } else {
                val c = reg.creator.pop()
                c.init(fixtureB, indexB, fixtureA, indexA)
                c
            }
        } else {
            return null
        }
    }

    fun pushContact(contact: Contact) {
        val fixtureA = contact.fixtureA
        val fixtureB = contact.fixtureB
        if (contact.m_manifold.pointCount > 0 && !fixtureA.isSensor && !fixtureB.isSensor) {
            fixtureA.body.isAwake = true
            fixtureB.body.isAwake = true
        }
        val type1 = fixtureA.type
        val type2 = fixtureB.type
        val creator = contactStacks[type1.ordinal][type2.ordinal]!!.creator
        creator.push(contact)
    }

    /**
     * Register a destruction listener. The listener is owned by you and must remain in scope.
     *
     * @param listener
     */
    fun setDestructionListener(listener: DestructionListener?) {
        destructionListener = listener
    }

    /**
     * Register a contact filter to provide specific control over collision. Otherwise the default
     * filter is used (_defaultFilter). The listener is owned by you and must remain in scope.
     *
     * @param filter
     */
    fun setContactFilter(filter: ContactFilter?) {
        m_contactManager.m_contactFilter = filter
    }

    /**
     * Register a contact event listener. The listener is owned by you and must remain in scope.
     *
     * @param listener
     */
    fun setContactListener(listener: ContactListener?) {
        m_contactManager.m_contactListener = listener
    }

    /**
     * Register a routine for debug drawing. The debug draw functions are called inside with
     * World.DrawDebugData method. The debug draw object is owned by you and must remain in scope.
     *
     * @param debugDraw
     */
    fun setDebugDraw(debugDraw: DebugDraw?) {
        this.debugDraw = debugDraw
    }

    /**
     * create a rigid body given a definition. No reference to the definition is retained.
     *
     * @warning This function is locked during callbacks.
     * @param def
     * @return
     */
    fun createBody(def: BodyDef?): Body? {
        assert(isLocked == false)
        if (isLocked) {
            return null
        }
        // TODO djm pooling
        val b = Body(def, this)

        // add to world doubly linked list
        b.m_prev = null
        b.m_next = m_bodyList
        if (m_bodyList != null) {
            m_bodyList!!.m_prev = b
        }
        m_bodyList = b
        ++m_bodyCount
        return b
    }

    /**
     * destroy a rigid body given a definition. No reference to the definition is retained. This
     * function is locked during callbacks.
     *
     * @warning This automatically deletes all associated shapes and joints.
     * @warning This function is locked during callbacks.
     * @param body
     */
    fun destroyBody(body: Body) {
        assert(m_bodyCount > 0)
        assert(isLocked == false)
        if (isLocked) {
            return
        }

        // Delete the attached joints.
        var je = body.m_jointList
        while (je != null) {
            val je0 = je
            je = je.next
            if (destructionListener != null) {
                destructionListener!!.sayGoodbye(je0.joint)
            }
            destroyJoint(je0.joint)
            body.m_jointList = je
        }
        body.m_jointList = null

        // Delete the attached contacts.
        var ce = body.m_contactList
        while (ce != null) {
            val ce0 = ce
            ce = ce.next
            m_contactManager.destroy(ce0.contact)
        }
        body.m_contactList = null
        var f = body.m_fixtureList
        while (f != null) {
            val f0 = f
            f = f.m_next
            if (destructionListener != null) {
                destructionListener!!.sayGoodbye(f0)
            }
            f0.destroyProxies(m_contactManager.m_broadPhase)
            f0.destroy()
            // TODO djm recycle fixtures (here or in that destroy method)
            body.m_fixtureList = f
            body.m_fixtureCount -= 1
        }
        body.m_fixtureList = null
        body.m_fixtureCount = 0

        // Remove world body list.
        if (body.m_prev != null) {
            body.m_prev!!.m_next = body.m_next
        }
        if (body.m_next != null) {
            body.m_next!!.m_prev = body.m_prev
        }
        if (body === m_bodyList) {
            m_bodyList = body.m_next
        }
        --m_bodyCount
        // TODO djm recycle body
    }

    /**
     * create a joint to constrain bodies together. No reference to the definition is retained. This
     * may cause the connected bodies to cease colliding.
     *
     * @warning This function is locked during callbacks.
     * @param def
     * @return
     */
    fun createJoint(def: JointDef): Joint? {
        assert(isLocked == false)
        if (isLocked) {
            return null
        }
        val j = Joint.create(this, def)

        // Connect to the world list.
        j.m_prev = null
        j.m_next = m_jointList
        if (m_jointList != null) {
            m_jointList!!.m_prev = j
        }
        m_jointList = j
        ++m_jointCount

        // Connect to the bodies' doubly linked lists.
        j.m_edgeA.joint = j
        j.m_edgeA.other = j.bodyB
        j.m_edgeA.prev = null
        j.m_edgeA.next = j.bodyA.m_jointList
        if (j.bodyA.m_jointList != null) {
            j.bodyA.m_jointList!!.prev = j.m_edgeA
        }
        j.bodyA.m_jointList = j.m_edgeA
        j.m_edgeB.joint = j
        j.m_edgeB.other = j.bodyA
        j.m_edgeB.prev = null
        j.m_edgeB.next = j.bodyB.m_jointList
        if (j.bodyB.m_jointList != null) {
            j.bodyB.m_jointList!!.prev = j.m_edgeB
        }
        j.bodyB.m_jointList = j.m_edgeB
        val bodyA = def.bodyA
        val bodyB = def.bodyB

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (def.collideConnected == false) {
            var edge = bodyB.contactList
            while (edge != null) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact!!.flagForFiltering()
                }
                edge = edge.next
            }
        }

        // Note: creating a joint doesn't wake the bodies.
        return j
    }

    /**
     * destroy a joint. This may cause the connected bodies to begin colliding.
     *
     * @warning This function is locked during callbacks.
     * @param joint
     */
    fun destroyJoint(j: Joint) {
        assert(isLocked == false)
        if (isLocked) {
            return
        }
        val collideConnected = j.collideConnected

        // Remove from the doubly linked list.
        if (j.m_prev != null) {
            j.m_prev!!.m_next = j.m_next
        }
        if (j.m_next != null) {
            j.m_next!!.m_prev = j.m_prev
        }
        if (j === m_jointList) {
            m_jointList = j.m_next
        }

        // Disconnect from island graph.
        val bodyA = j.bodyA
        val bodyB = j.bodyB

        // Wake up connected bodies.
        bodyA.isAwake = true
        bodyB.isAwake = true

        // Remove from body 1.
        if (j.m_edgeA.prev != null) {
            j.m_edgeA.prev!!.next = j.m_edgeA.next
        }
        if (j.m_edgeA.next != null) {
            j.m_edgeA.next!!.prev = j.m_edgeA.prev
        }
        if (j.m_edgeA === bodyA.m_jointList) {
            bodyA.m_jointList = j.m_edgeA.next
        }
        j.m_edgeA.prev = null
        j.m_edgeA.next = null

        // Remove from body 2
        if (j.m_edgeB.prev != null) {
            j.m_edgeB.prev!!.next = j.m_edgeB.next
        }
        if (j.m_edgeB.next != null) {
            j.m_edgeB.next!!.prev = j.m_edgeB.prev
        }
        if (j.m_edgeB === bodyB.m_jointList) {
            bodyB.m_jointList = j.m_edgeB.next
        }
        j.m_edgeB.prev = null
        j.m_edgeB.next = null
        Joint.destroy(j)
        assert(m_jointCount > 0)
        --m_jointCount

        // If the joint prevents collisions, then flag any contacts for filtering.
        if (collideConnected == false) {
            var edge = bodyB.contactList
            while (edge != null) {
                if (edge.other === bodyA) {
                    // Flag the contact for filtering at the next time step (where either
                    // body is awake).
                    edge.contact!!.flagForFiltering()
                }
                edge = edge.next
            }
        }
    }

    // djm pooling
    private val step = TimeStep()
    private val stepTimer = Timer()
    private val tempTimer = Timer()

    /**
     * Take a time step. This performs collision detection, integration, and constraint solution.
     *
     * @param timeStep the amount of time to simulate, this should not vary.
     * @param velocityIterations for the velocity constraint solver.
     * @param positionIterations for the position constraint solver.
     */
    fun step(dt: Float, velocityIterations: Int, positionIterations: Int) {
        stepTimer.reset()
        tempTimer.reset()
        // log.debug("Starting step");
        // If new fixtures were added, we need to find the new contacts.
        if (m_flags and NEW_FIXTURE == NEW_FIXTURE) {
            // log.debug("There's a new fixture, lets look for new contacts");
            m_contactManager.findNewContacts()
            m_flags = m_flags and NEW_FIXTURE.inv()
        }
        m_flags = m_flags or LOCKED
        step.dt = dt
        step.velocityIterations = velocityIterations
        step.positionIterations = positionIterations
        step.inv_dt = if (dt > 0.0f) {
            1.0f / dt
        } else {
            0.0f
        }
        step.dtRatio = m_inv_dt0 * dt
        step.warmStarting = isWarmStarting
        m_profile.stepInit.record(tempTimer.milliseconds)

        // Update contacts. This is where some contacts are destroyed.
        tempTimer.reset()
        m_contactManager.collide()
        m_profile.collide.record(tempTimer.milliseconds)

        // Integrate velocities, solve velocity constraints, and integrate positions.
        if (m_stepComplete && step.dt > 0.0f) {
            tempTimer.reset()
            m_particleSystem.solve(step) // Particle Simulation
            m_profile.solveParticleSystem.record(tempTimer.milliseconds)
            tempTimer.reset()
            solve(step)
            m_profile.solve.record(tempTimer.milliseconds)
        }

        // Handle TOI events.
        if (isContinuousPhysics && step.dt > 0.0f) {
            tempTimer.reset()
            solveTOI(step)
            m_profile.solveTOI.record(tempTimer.milliseconds)
        }
        if (step.dt > 0.0f) {
            m_inv_dt0 = step.inv_dt
        }
        if (m_flags and CLEAR_FORCES == CLEAR_FORCES) {
            clearForces()
        }
        m_flags = m_flags and LOCKED.inv()
        // log.debug("ending step");
        m_profile.step.record(stepTimer.milliseconds)
    }

    /**
     * Call this after you are done with time steps to clear the forces. You normally call this after
     * each call to Step, unless you are performing sub-steps. By default, forces will be
     * automatically cleared, so you don't need to call this function.
     *
     * @see setAutoClearForces
     */
    fun clearForces() {
        var body = m_bodyList
        while (body != null) {
            body.m_force.setZero()
            body.m_torque = 0.0f
            body = body.next
        }
    }

    private val color = Color3f()
    private val xf = Transform()
    private val cA = Vec2()
    private val cB = Vec2()
    private val avs = Vec2Array()

    /**
     * Call this to draw shapes and other debug draw data.
     */
    fun drawDebugData() {
        if (debugDraw == null) {
            return
        }
        val flags = debugDraw!!.flags
        val wireframe = flags and DebugDraw.Companion.e_wireframeDrawingBit != 0
        if (flags and DebugDraw.Companion.e_shapeBit != 0) {
            var b = m_bodyList
            while (b != null) {
                xf.set(b.transform)
                var f = b.fixtureList
                while (f != null) {
                    if (b.isActive == false) {
                        color.set(0.5f, 0.5f, 0.3f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.type === BodyType.STATIC) {
                        color.set(0.5f, 0.9f, 0.3f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.type === BodyType.KINEMATIC) {
                        color.set(0.5f, 0.5f, 0.9f)
                        drawShape(f, xf, color, wireframe)
                    } else if (b.isAwake == false) {
                        color.set(0.5f, 0.5f, 0.5f)
                        drawShape(f, xf, color, wireframe)
                    } else {
                        color.set(0.9f, 0.7f, 0.7f)
                        drawShape(f, xf, color, wireframe)
                    }
                    f = f.m_next
                }
                b = b.next
            }
            drawParticleSystem(m_particleSystem)
        }
        if (flags and DebugDraw.Companion.e_jointBit != 0) {
            var j = m_jointList
            while (j != null) {
                drawJoint(j)
                j = j.next
            }
        }
        if (flags and DebugDraw.Companion.e_pairBit != 0) {
            color.set(0.3f, 0.9f, 0.9f)
            var c = m_contactManager.m_contactList
            while (c != null) {
                val fixtureA = c.fixtureA
                val fixtureB = c.fixtureB
                fixtureA.getAABB(c.childIndexA)!!.getCenterToOut(cA)
                fixtureB.getAABB(c.childIndexB)!!.getCenterToOut(cB)
                debugDraw!!.drawSegment(cA, cB, color)
                c = c.next
            }
        }
        if (flags and DebugDraw.Companion.e_aabbBit != 0) {
            color.set(0.9f, 0.3f, 0.9f)
            var b = m_bodyList
            while (b != null) {
                if (b.isActive == false) {
                    b = b.next
                    continue
                }
                var f = b.fixtureList
                while (f != null) {
                    for (i in 0 until f.m_proxyCount) {
                        val proxy = f.m_proxies!![i]
                        val aabb = m_contactManager.m_broadPhase.getFatAABB(proxy.proxyId)
                        if (aabb != null) {
                            val vs = avs.get(4)
                            vs[0].set(aabb.lowerBound.x, aabb.lowerBound.y)
                            vs[1].set(aabb.upperBound.x, aabb.lowerBound.y)
                            vs[2].set(aabb.upperBound.x, aabb.upperBound.y)
                            vs[3].set(aabb.lowerBound.x, aabb.upperBound.y)
                            debugDraw!!.drawPolygon(vs, 4, color)
                        }
                    }
                    f = f.m_next
                }
                b = b.next
            }
        }
        if (flags and DebugDraw.Companion.e_centerOfMassBit != 0) {
            var b = m_bodyList
            while (b != null) {
                xf.set(b.transform)
                xf.p.set(b.worldCenter)
                debugDraw!!.drawTransform(xf)
                b = b.next
            }
        }
        if (flags and DebugDraw.Companion.e_dynamicTreeBit != 0) {
            m_contactManager.m_broadPhase.drawTree(debugDraw)
        }
        debugDraw!!.flush()
    }

    private val wqwrapper = WorldQueryWrapper()

    /**
     * Query the world for all fixtures that potentially overlap the provided AABB.
     *
     * @param callback a user implemented callback class.
     * @param aabb the query box.
     */
    fun queryAABB(callback: QueryCallback?, aabb: AABB?) {
        wqwrapper.broadPhase = m_contactManager.m_broadPhase
        wqwrapper.callback = callback
        m_contactManager.m_broadPhase.query(wqwrapper, aabb)
    }

    /**
     * Query the world for all fixtures and particles that potentially overlap the provided AABB.
     *
     * @param callback a user implemented callback class.
     * @param particleCallback callback for particles.
     * @param aabb the query box.
     */
    fun queryAABB(
        callback: QueryCallback?, particleCallback: ParticleQueryCallback?,
        aabb: AABB?
    ) {
        wqwrapper.broadPhase = m_contactManager.m_broadPhase
        wqwrapper.callback = callback
        m_contactManager.m_broadPhase.query(wqwrapper, aabb)
        m_particleSystem.queryAABB(particleCallback, aabb)
    }

    /**
     * Query the world for all particles that potentially overlap the provided AABB.
     *
     * @param particleCallback callback for particles.
     * @param aabb the query box.
     */
    fun queryAABB(particleCallback: ParticleQueryCallback?, aabb: AABB?) {
        m_particleSystem.queryAABB(particleCallback, aabb)
    }

    private val wrcwrapper = WorldRayCastWrapper()
    private val input = RayCastInput()

    /**
     * Ray-cast the world for all fixtures in the path of the ray. Your callback controls whether you
     * get the closest point, any point, or n-points. The ray-cast ignores shapes that contain the
     * starting point.
     *
     * @param callback a user implemented callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(callback: RayCastCallback?, point1: Vec2, point2: Vec2) {
        wrcwrapper.broadPhase = m_contactManager.m_broadPhase
        wrcwrapper.callback = callback
        input.maxFraction = 1.0f
        input.p1.set(point1)
        input.p2.set(point2)
        m_contactManager.m_broadPhase.raycast(wrcwrapper, input)
    }

    /**
     * Ray-cast the world for all fixtures and particles in the path of the ray. Your callback
     * controls whether you get the closest point, any point, or n-points. The ray-cast ignores shapes
     * that contain the starting point.
     *
     * @param callback a user implemented callback class.
     * @param particleCallback the particle callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(
        callback: RayCastCallback?, particleCallback: ParticleRaycastCallback?,
        point1: Vec2, point2: Vec2
    ) {
        wrcwrapper.broadPhase = m_contactManager.m_broadPhase
        wrcwrapper.callback = callback
        input.maxFraction = 1.0f
        input.p1.set(point1)
        input.p2.set(point2)
        m_contactManager.m_broadPhase.raycast(wrcwrapper, input)
        m_particleSystem.raycast(particleCallback, point1, point2)
    }

    /**
     * Ray-cast the world for all particles in the path of the ray. Your callback controls whether you
     * get the closest point, any point, or n-points.
     *
     * @param particleCallback the particle callback class.
     * @param point1 the ray starting point
     * @param point2 the ray ending point
     */
    fun raycast(particleCallback: ParticleRaycastCallback?, point1: Vec2, point2: Vec2) {
        m_particleSystem.raycast(particleCallback, point1, point2)
    }

    /**
     * Get the world body list. With the returned body, use Body.getNext to get the next body in the
     * world list. A null body indicates the end of the list.
     *
     * @return the head of the world body list.
     */
    fun getBodyList(): Body? {
        return m_bodyList
    }

    /**
     * Get the world joint list. With the returned joint, use Joint.getNext to get the next joint in
     * the world list. A null joint indicates the end of the list.
     *
     * @return the head of the world joint list.
     */
    fun getJointList(): Joint? {
        return m_jointList
    }

    /**
     * Get the world contact list. With the returned contact, use Contact.getNext to get the next
     * contact in the world list. A null contact indicates the end of the list.
     *
     * @return the head of the world contact list.
     * @warning contacts are created and destroyed in the middle of a time step. Use ContactListener
     * to avoid missing contacts.
     */
    fun getContactList(): Contact? {
        return m_contactManager.m_contactList
    }

    /**
     * Enable/disable warm starting. For testing.
     *
     * @param flag
     */
    fun setWarmStarting(flag: Boolean) {
        isWarmStarting = flag
    }

    /**
     * Enable/disable continuous physics. For testing.
     *
     * @param flag
     */
    fun setContinuousPhysics(flag: Boolean) {
        isContinuousPhysics = flag
    }

    /**
     * Get the number of broad-phase proxies.
     *
     * @return
     */
    val proxyCount: Int
        get() = m_contactManager.m_broadPhase.proxyCount

    /**
     * Get the number of bodies.
     *
     * @return
     */
    val bodyCount: Int
        get() = m_bodyCount

    /**
     * Get the number of joints.
     *
     * @return
     */
    val jointCount: Int
        get() = m_jointCount

    /**
     * Get the number of contacts (each may have 0 or more contact points).
     *
     * @return
     */
    val contactCount: Int
        get() = m_contactManager.m_contactCount

    /**
     * Gets the height of the dynamic tree
     *
     * @return
     */
    val treeHeight: Int
        get() = m_contactManager.m_broadPhase.treeHeight

    /**
     * Gets the balance of the dynamic tree
     *
     * @return
     */
    val treeBalance: Int
        get() = m_contactManager.m_broadPhase.treeBalance

    /**
     * Gets the quality of the dynamic tree
     *
     * @return
     */
    val treeQuality: Float
        get() = m_contactManager.m_broadPhase.treeQuality

    /**
     * Change the global gravity vector.
     *
     * @param gravity
     */
    fun setGravity(gravity: Vec2) {
        m_gravity.set(gravity)
    }

    /**
     * Get the global gravity vector.
     *
     * @return
     */
    fun getGravity(): Vec2 {
        return m_gravity
    }

    /**
     * Is the world locked (in the middle of a time step).
     *
     * @return
     */
    val isLocked: Boolean
        get() = m_flags and LOCKED == LOCKED

    /**
     * Set flag to control automatic clearing of forces after each time step.
     *
     * @param flag
     */
    fun setAutoClearForces(flag: Boolean) {
        if (flag) {
            m_flags = m_flags or CLEAR_FORCES
        } else {
            m_flags = m_flags and CLEAR_FORCES.inv()
        }
    }

    /**
     * Get the flag that controls automatic clearing of forces after each time step.
     *
     * @return
     */
    val autoClearForces: Boolean
        get() = m_flags and CLEAR_FORCES == CLEAR_FORCES

    /**
     * Get the contact manager for testing purposes
     *
     * @return
     */
    fun getContactManager(): ContactManager {
        return m_contactManager
    }

    fun getProfile(): Profile {
        return m_profile
    }

    private val island = Island()
    private var stack = arrayOfNulls<Body>(10) // TODO djm find a good initial stack number;
    private val broadphaseTimer = Timer()
    private fun solve(step: TimeStep) {
        m_profile.solveInit.startAccum()
        m_profile.solveVelocity.startAccum()
        m_profile.solvePosition.startAccum()

        // update previous transforms
        var b = m_bodyList
        while (b != null) {
            b.m_xf0.set(b.m_xf)
            b = b.m_next
        }

        // Size the island for the worst case.
        island.init(
            m_bodyCount, m_contactManager.m_contactCount, m_jointCount,
            m_contactManager.m_contactListener
        )

        // Clear all the island flags.
        b = m_bodyList
        while (b != null) {
            b.m_flags = b.m_flags and Body.Companion.e_islandFlag.inv()
            b = b.m_next
        }
        var c = m_contactManager.m_contactList
        while (c != null) {
            c.m_flags = c.m_flags and Contact.Companion.ISLAND_FLAG.inv()
            c = c.m_next
        }
        var j = m_jointList
        while (j != null) {
            j.m_islandFlag = false
            j = j.m_next
        }

        // Build and simulate all awake islands.
        val stackSize = m_bodyCount
        if (stack.size < stackSize) {
            stack = arrayOfNulls(stackSize)
        }
        var seed = m_bodyList
        while (seed != null) {
            if (seed.m_flags and Body.Companion.e_islandFlag == Body.Companion.e_islandFlag) {
                seed = seed.m_next
                continue
            }
            if (seed.isAwake == false || seed.isActive == false) {
                seed = seed.m_next
                continue
            }

            // The seed can be dynamic or kinematic.
            if (seed.type === BodyType.STATIC) {
                seed = seed.m_next
                continue
            }

            // Reset island and stack.
            island.clear()
            var stackCount = 0
            stack[stackCount++] = seed
            seed.m_flags = seed.m_flags or Body.Companion.e_islandFlag

            // Perform a depth first search (DFS) on the constraint graph.
            while (stackCount > 0) {
                // Grab the next body off the stack and add it to the island.
                val b = stack[--stackCount]!!
                assert(b.isActive == true)
                island.add(b)

                // Make sure the body is awake.
                b.isAwake = true

                // To keep islands as small as possible, we don't
                // propagate islands across static bodies.
                if (b.type === BodyType.STATIC) {
                    continue
                }

                // Search all contacts connected to this body.
                var ce = b.m_contactList
                while (ce != null) {
                    val contact = ce.contact!!

                    // Has this contact already been added to an island?
                    if (contact.m_flags and Contact.Companion.ISLAND_FLAG == Contact.Companion.ISLAND_FLAG) {
                        ce = ce.next
                        continue
                    }

                    // Is this contact solid and touching?
                    if (contact.isEnabled == false || contact.isTouching == false) {
                        ce = ce.next
                        continue
                    }

                    // Skip sensors.
                    val sensorA = contact.m_fixtureA!!.m_isSensor
                    val sensorB = contact.m_fixtureB!!.m_isSensor
                    if (sensorA || sensorB) {
                        ce = ce.next
                        continue
                    }
                    island.add(contact)
                    contact.m_flags = contact.m_flags or Contact.Companion.ISLAND_FLAG
                    val other = ce.other

                    // Was the other body already added to this island?
                    if (other!!.m_flags and Body.Companion.e_islandFlag == Body.Companion.e_islandFlag) {
                        ce = ce.next
                        continue
                    }
                    assert(stackCount < stackSize)
                    stack[stackCount++] = other
                    other.m_flags = other.m_flags or Body.Companion.e_islandFlag
                    ce = ce.next
                }

                // Search all joints connect to this body.
                var je = b.m_jointList
                while (je != null) {
                    if (je.joint!!.m_islandFlag == true) {
                        je = je.next
                        continue
                    }
                    val other = je.other

                    // Don't simulate joints connected to inactive bodies.
                    if (other!!.isActive == false) {
                        je = je.next
                        continue
                    }
                    island.add(je.joint)
                    je.joint!!.m_islandFlag = true
                    if (other.m_flags and Body.Companion.e_islandFlag == Body.Companion.e_islandFlag) {
                        je = je.next
                        continue
                    }
                    assert(stackCount < stackSize)
                    stack[stackCount++] = other
                    other.m_flags = other.m_flags or Body.Companion.e_islandFlag
                    je = je.next
                }
            }
            island.solve(m_profile, step, m_gravity, isSleepingAllowed)

            // Post solve cleanup.
            for (i in 0 until island.m_bodyCount) {
                // Allow static bodies to participate in other islands.
                val b = island.m_bodies[i]!!
                if (b.type === BodyType.STATIC) {
                    b.m_flags = b.m_flags and Body.Companion.e_islandFlag.inv()
                }
            }
            seed = seed.m_next
        }
        m_profile.solveInit.endAccum()
        m_profile.solveVelocity.endAccum()
        m_profile.solvePosition.endAccum()
        broadphaseTimer.reset()
        // Synchronize fixtures, check for out of range bodies.
        b = m_bodyList
        while (b != null) {

            // If a body was not in an island then it did not move.
            if (b.m_flags and Body.Companion.e_islandFlag == 0) {
                b = b.next
                continue
            }
            if (b.type === BodyType.STATIC) {
                b = b.next
                continue
            }

            // Update fixtures (for broad-phase).
            b.synchronizeFixtures()
            b = b.next
        }

        // Look for new contacts.
        m_contactManager.findNewContacts()
        m_profile.broadphase.record(broadphaseTimer.milliseconds)
    }

    private val toiIsland = Island()
    private val toiInput = TOIInput()
    private val toiOutput = TOIOutput()
    private val subStep = TimeStep()
    private val tempBodies = arrayOfNulls<Body>(2)
    private val backup1 = Sweep()
    private val backup2 = Sweep()
    private fun solveTOI(step: TimeStep) {
        val island = toiIsland
        island.init(
            2 * Settings.maxTOIContacts, Settings.maxTOIContacts, 0,
            m_contactManager.m_contactListener
        )
        if (m_stepComplete) {
            var b = m_bodyList
            while (b != null) {
                b.m_flags = b.m_flags and Body.Companion.e_islandFlag.inv()
                b.m_sweep.alpha0 = 0.0f
                b = b.m_next
            }
            var c = m_contactManager.m_contactList
            while (c != null) {
                // Invalidate TOI
                c.m_flags = c.m_flags and (Contact.Companion.TOI_FLAG or Contact.Companion.ISLAND_FLAG).inv()
                c.m_toiCount = 0
                c.m_toi = 1.0f
                c = c.m_next
            }
        }

        // Find TOI events and solve them.
        while (true) {
            // Find the first TOI.
            var minContact: Contact? = null
            var minAlpha = 1.0f
            var c = m_contactManager.m_contactList
            while (c != null) {

                // Is this contact disabled?
                if (c.isEnabled == false) {
                    c = c.m_next
                    continue
                }

                // Prevent excessive sub-stepping.
                if (c.m_toiCount > Settings.maxSubSteps) {
                    c = c.m_next
                    continue
                }
                var alpha = 1.0f
                if (c.m_flags and Contact.Companion.TOI_FLAG != 0) {
                    // This contact has a valid cached TOI.
                    alpha = c.m_toi
                } else {
                    val fA = c.fixtureA
                    val fB = c.fixtureB

                    // Is there a sensor?
                    if (fA!!.isSensor || fB!!.isSensor) {
                        c = c.m_next
                        continue
                    }
                    val bA = fA.body
                    val bB = fB.body
                    val typeA = bA.m_type
                    val typeB = bB.m_type
                    assert(typeA === BodyType.DYNAMIC || typeB === BodyType.DYNAMIC)
                    val activeA = bA.isAwake && typeA !== BodyType.STATIC
                    val activeB = bB.isAwake && typeB !== BodyType.STATIC

                    // Is at least one body active (awake and dynamic or kinematic)?
                    if (activeA == false && activeB == false) {
                        c = c.m_next
                        continue
                    }
                    val collideA = bA.isBullet || typeA !== BodyType.DYNAMIC
                    val collideB = bB.isBullet || typeB !== BodyType.DYNAMIC

                    // Are these two non-bullet dynamic bodies?
                    if (collideA == false && collideB == false) {
                        c = c.m_next
                        continue
                    }

                    // Compute the TOI for this contact.
                    // Put the sweeps onto the same time interval.
                    var alpha0 = bA.m_sweep.alpha0
                    if (bA.m_sweep.alpha0 < bB.m_sweep.alpha0) {
                        alpha0 = bB.m_sweep.alpha0
                        bA.m_sweep.advance(alpha0)
                    } else if (bB.m_sweep.alpha0 < bA.m_sweep.alpha0) {
                        alpha0 = bA.m_sweep.alpha0
                        bB.m_sweep.advance(alpha0)
                    }
                    assert(alpha0 < 1.0f)
                    val indexA = c.childIndexA
                    val indexB = c.childIndexB

                    // Compute the time of impact in interval [0, minTOI]
                    val input = toiInput
                    input.proxyA.set(fA.shape, indexA)
                    input.proxyB.set(fB.shape, indexB)
                    input.sweepA.set(bA.m_sweep)
                    input.sweepB.set(bB.m_sweep)
                    input.tMax = 1.0f
                    pool.timeOfImpact.timeOfImpact(toiOutput, input)

                    // Beta is the fraction of the remaining portion of the .
                    val beta = toiOutput.t
                    alpha = if (toiOutput.state === TOIOutputState.TOUCHING) {
                        MathUtils.min(alpha0 + (1.0f - alpha0) * beta, 1.0f)
                    } else {
                        1.0f
                    }
                    c.m_toi = alpha
                    c.m_flags = c.m_flags or Contact.Companion.TOI_FLAG
                }
                if (alpha < minAlpha) {
                    // This is the minimum TOI found so far.
                    minContact = c
                    minAlpha = alpha
                }
                c = c.m_next
            }
            if (minContact == null || 1.0f - 10.0f * Settings.EPSILON < minAlpha) {
                // No more TOI events. Done!
                m_stepComplete = true
                break
            }

            // Advance the bodies to the TOI.
            val fA = minContact.fixtureA
            val fB = minContact.fixtureB
            val bA = fA!!.body
            val bB = fB!!.body
            backup1.set(bA.m_sweep)
            backup2.set(bB.m_sweep)
            bA.advance(minAlpha)
            bB.advance(minAlpha)

            // The TOI contact likely has some new contact points.
            minContact.update(m_contactManager.m_contactListener)
            minContact.m_flags = minContact.m_flags and Contact.Companion.TOI_FLAG.inv()
            ++minContact.m_toiCount

            // Is the contact solid?
            if (minContact.isEnabled == false || minContact.isTouching == false) {
                // Restore the sweeps.
                minContact.isEnabled = false
                bA.m_sweep.set(backup1)
                bB.m_sweep.set(backup2)
                bA.synchronizeTransform()
                bB.synchronizeTransform()
                continue
            }
            bA.isAwake = true
            bB.isAwake = true

            // Build the island
            island.clear()
            island.add(bA)
            island.add(bB)
            island.add(minContact)
            bA.m_flags = bA.m_flags or Body.Companion.e_islandFlag
            bB.m_flags = bB.m_flags or Body.Companion.e_islandFlag
            minContact.m_flags = minContact.m_flags or Contact.Companion.ISLAND_FLAG

            // Get contacts on bodyA and bodyB.
            tempBodies[0] = bA
            tempBodies[1] = bB
            for (i in 0..1) {
                val body = tempBodies[i]!!
                if (body.m_type === BodyType.DYNAMIC) {
                    var ce = body.m_contactList
                    while (ce != null) {
                        if (island.m_bodyCount == island.m_bodyCapacity) {
                            break
                        }
                        if (island.m_contactCount == island.m_contactCapacity) {
                            break
                        }
                        val contact = ce.contact!!

                        // Has this contact already been added to the island?
                        if (contact.m_flags and Contact.Companion.ISLAND_FLAG != 0) {
                            ce = ce.next
                            continue
                        }

                        // Only add static, kinematic, or bullet bodies.
                        val other = ce.other
                        if (other!!.m_type === BodyType.DYNAMIC && body.isBullet == false && other.isBullet == false) {
                            ce = ce.next
                            continue
                        }

                        // Skip sensors.
                        val sensorA = contact.m_fixtureA!!.m_isSensor
                        val sensorB = contact.m_fixtureB!!.m_isSensor
                        if (sensorA || sensorB) {
                            ce = ce.next
                            continue
                        }

                        // Tentatively advance the body to the TOI.
                        backup1.set(other.m_sweep)
                        if (other.m_flags and Body.Companion.e_islandFlag == 0) {
                            other.advance(minAlpha)
                        }

                        // Update the contact points
                        contact.update(m_contactManager.m_contactListener)

                        // Was the contact disabled by the user?
                        if (contact.isEnabled == false) {
                            other.m_sweep.set(backup1)
                            other.synchronizeTransform()
                            ce = ce.next
                            continue
                        }

                        // Are there contact points?
                        if (contact.isTouching == false) {
                            other.m_sweep.set(backup1)
                            other.synchronizeTransform()
                            ce = ce.next
                            continue
                        }

                        // Add the contact to the island
                        contact.m_flags = contact.m_flags or Contact.Companion.ISLAND_FLAG
                        island.add(contact)

                        // Has the other body already been added to the island?
                        if (other.m_flags and Body.Companion.e_islandFlag != 0) {
                            ce = ce.next
                            continue
                        }

                        // Add the other body to the island.
                        other.m_flags = other.m_flags or Body.Companion.e_islandFlag
                        if (other.m_type !== BodyType.STATIC) {
                            other.isAwake = true
                        }
                        island.add(other)
                        ce = ce.next
                    }
                }
            }
            subStep.dt = (1.0f - minAlpha) * step.dt
            subStep.inv_dt = 1.0f / subStep.dt
            subStep.dtRatio = 1.0f
            subStep.positionIterations = 20
            subStep.velocityIterations = step.velocityIterations
            subStep.warmStarting = false
            island.solveTOI(subStep, bA.m_islandIndex, bB.m_islandIndex)

            // Reset island flags and synchronize broad-phase proxies.
            for (i in 0 until island.m_bodyCount) {
                val body = island.m_bodies[i]!!
                body.m_flags = body.m_flags and Body.Companion.e_islandFlag.inv()
                if (body.m_type !== BodyType.DYNAMIC) {
                    continue
                }
                body.synchronizeFixtures()

                // Invalidate all contact TOIs on this displaced body.
                var ce = body.m_contactList
                while (ce != null) {
                    ce.contact!!.m_flags =
                        ce.contact!!.m_flags and (Contact.Companion.TOI_FLAG or Contact.Companion.ISLAND_FLAG).inv()
                    ce = ce.next
                }
            }

            // Commit fixture proxy movements to the broad-phase so that new contacts are created.
            // Also, some contacts can be destroyed.
            m_contactManager.findNewContacts()
            if (isSubStepping) {
                m_stepComplete = false
                break
            }
        }
    }

    private fun drawJoint(joint: Joint) {
        val bodyA = joint.bodyA
        val bodyB = joint.bodyB
        val xf1 = bodyA.transform
        val xf2 = bodyB.transform
        val x1 = xf1.p
        val x2 = xf2.p
        val p1 = pool.popVec2()
        val p2 = pool.popVec2()
        joint.getAnchorA(p1)
        joint.getAnchorB(p2)
        color.set(0.5f, 0.8f, 0.8f)
        when (joint.type) {
            DISTANCE -> debugDraw!!.drawSegment(p1, p2, color)
            PULLEY -> {
                val pulley = joint as PulleyJoint
                val s1 = pulley.groundAnchorA
                val s2 = pulley.groundAnchorB
                debugDraw!!.drawSegment(s1, p1, color)
                debugDraw!!.drawSegment(s2, p2, color)
                debugDraw!!.drawSegment(s1, s2, color)
            }
            CONSTANT_VOLUME, MOUSE -> {
            }
            else -> {
                debugDraw!!.drawSegment(x1, p1, color)
                debugDraw!!.drawSegment(p1, p2, color)
                debugDraw!!.drawSegment(x2, p2, color)
            }
        }
        pool.pushVec2(2)
    }

    // NOTE this corresponds to the liquid test, so the debugdraw can draw
    // the liquid particles correctly. They should be the same.
    private val liquidOffset = Vec2()
    private val circCenterMoved = Vec2()
    private val liquidColor = Color3f(.4f, .4f, 1f)
    private val center = Vec2()
    private val axis = Vec2()
    private val v1 = Vec2()
    private val v2 = Vec2()
    private val tlvertices = Vec2Array()
    private fun drawShape(fixture: Fixture, xf: Transform, color: Color3f, wireframe: Boolean) {
        when (fixture.type) {
            ShapeType.CIRCLE -> {
                val circle = fixture.shape as CircleShape

                // Vec2 center = Mul(xf, circle.m_p);
                Transform.mulToOutUnsafe(xf, circle.m_p, center)
                val radius = circle.m_radius
                xf.q.getXAxis(axis)
                if (fixture.userData != null && fixture.userData == LIQUID_INT) {
                    val b = fixture.body
                    liquidOffset.set(b.m_linearVelocity)
                    val linVelLength = b.m_linearVelocity.length()
                    if (averageLinearVel == -1f) {
                        averageLinearVel = linVelLength
                    } else {
                        averageLinearVel = .98f * averageLinearVel + .02f * linVelLength
                    }
                    liquidOffset.mulLocal(liquidLength / averageLinearVel / 2)
                    circCenterMoved.set(center).addLocal(liquidOffset)
                    center.subLocal(liquidOffset)
                    debugDraw!!.drawSegment(center, circCenterMoved, liquidColor)
                    return
                }
                if (wireframe) {
                    debugDraw!!.drawCircle(center, radius, axis, color)
                } else {
                    debugDraw!!.drawSolidCircle(center, radius, axis, color)
                }
            }
            ShapeType.POLYGON -> {
                val poly = fixture.shape as PolygonShape
                val vertexCount = poly.m_count
                assert(vertexCount <= Settings.maxPolygonVertices)
                val vertices = tlvertices.get(Settings.maxPolygonVertices)
                for (i in 0 until vertexCount) {
                    // vertices[i] = Mul(xf, poly.m_vertices[i]);
                    Transform.mulToOutUnsafe(xf, poly.m_vertices[i], vertices[i])
                }
                if (wireframe) {
                    debugDraw!!.drawPolygon(vertices, vertexCount, color)
                } else {
                    debugDraw!!.drawSolidPolygon(vertices, vertexCount, color)
                }
            }
            ShapeType.EDGE -> {
                val edge = fixture.shape as EdgeShape
                Transform.mulToOutUnsafe(xf, edge.m_vertex1, v1)
                Transform.mulToOutUnsafe(xf, edge.m_vertex2, v2)
                debugDraw!!.drawSegment(v1, v2, color)
            }
            ShapeType.CHAIN -> {
                val chain = fixture.shape as ChainShape
                val count = chain.m_count
                val vertices = chain.m_vertices
                Transform.mulToOutUnsafe(xf, vertices[0], v1)
                for (i in 1 until count) {
                    Transform.mulToOutUnsafe(xf, vertices[i], v2)
                    debugDraw!!.drawSegment(v1, v2, color)
                    debugDraw!!.drawCircle(v1, 0.05f, color)
                    v1.set(v2)
                }
            }
            else -> {
            }
        }
    }

    private fun drawParticleSystem(system: ParticleSystem) {
        val wireframe = debugDraw!!.flags and DebugDraw.Companion.e_wireframeDrawingBit != 0
        val particleCount = system.particleCount
        if (particleCount != 0) {
            val particleRadius = system.particleRadius
            val positionBuffer = system.particlePositionBuffer
            var colorBuffer: Array<ParticleColor?>? = null
            if (system.m_colorBuffer.data != null) {
                colorBuffer = system.particleColorBuffer
            }
            if (wireframe) {
                debugDraw!!.drawParticlesWireframe(
                    positionBuffer, particleRadius, colorBuffer,
                    particleCount
                )
            } else {
                debugDraw!!.drawParticles(positionBuffer, particleRadius, colorBuffer, particleCount)
            }
        }
    }

    /**
     * Create a particle whose properties have been defined. No reference to the definition is
     * retained. A simulation step must occur before it's possible to interact with a newly created
     * particle. For example, DestroyParticleInShape() will not destroy a particle until Step() has
     * been called.
     *
     * @warning This function is locked during callbacks.
     * @return the index of the particle.
     */
    fun createParticle(def: ParticleDef?): Int {
        assert(isLocked == false)
        if (isLocked) {
            return 0
        }
        val p = m_particleSystem.createParticle(def)
        return p
    }

    /**
     * Destroy a particle. The particle is removed after the next step.
     *
     * @param index
     */
    fun destroyParticle(index: Int) {
        destroyParticle(index, false)
    }

    /**
     * Destroy a particle. The particle is removed after the next step.
     *
     * @param Index of the particle to destroy.
     * @param Whether to call the destruction listener just before the particle is destroyed.
     */
    fun destroyParticle(index: Int, callDestructionListener: Boolean) {
        m_particleSystem.destroyParticle(index, callDestructionListener)
    }

    /**
     * Destroy particles inside a shape without enabling the destruction callback for destroyed
     * particles. This function is locked during callbacks. For more information see
     * DestroyParticleInShape(Shape&, Transform&,bool).
     *
     * @param Shape which encloses particles that should be destroyed.
     * @param Transform applied to the shape.
     * @warning This function is locked during callbacks.
     * @return Number of particles destroyed.
     */
    fun destroyParticlesInShape(shape: Shape?, xf: Transform?): Int {
        return destroyParticlesInShape(shape, xf, false)
    }

    /**
     * Destroy particles inside a shape. This function is locked during callbacks. In addition, this
     * function immediately destroys particles in the shape in contrast to DestroyParticle() which
     * defers the destruction until the next simulation step.
     *
     * @param Shape which encloses particles that should be destroyed.
     * @param Transform applied to the shape.
     * @param Whether to call the world b2DestructionListener for each particle destroyed.
     * @warning This function is locked during callbacks.
     * @return Number of particles destroyed.
     */
    fun destroyParticlesInShape(
        shape: Shape?, xf: Transform?,
        callDestructionListener: Boolean
    ): Int {
        assert(isLocked == false)
        if (isLocked) {
            return 0
        }
        return m_particleSystem.destroyParticlesInShape(shape, xf, callDestructionListener)
    }

    /**
     * Create a particle group whose properties have been defined. No reference to the definition is
     * retained.
     *
     * @warning This function is locked during callbacks.
     */
    fun createParticleGroup(def: ParticleGroupDef?): ParticleGroup? {
        assert(isLocked == false)
        if (isLocked) {
            return null
        }
        val g = m_particleSystem.createParticleGroup(def)
        return g
    }

    /**
     * Join two particle groups.
     *
     * @param the first group. Expands to encompass the second group.
     * @param the second group. It is destroyed.
     * @warning This function is locked during callbacks.
     */
    fun joinParticleGroups(groupA: ParticleGroup?, groupB: ParticleGroup?) {
        assert(isLocked == false)
        if (isLocked) {
            return
        }
        m_particleSystem.joinParticleGroups(groupA, groupB)
    }

    /**
     * Destroy particles in a group. This function is locked during callbacks.
     *
     * @param The particle group to destroy.
     * @param Whether to call the world b2DestructionListener for each particle is destroyed.
     * @warning This function is locked during callbacks.
     */
    fun destroyParticlesInGroup(group: ParticleGroup?, callDestructionListener: Boolean) {
        assert(isLocked == false)
        if (isLocked) {
            return
        }
        m_particleSystem.destroyParticlesInGroup(group, callDestructionListener)
    }

    /**
     * Destroy particles in a group without enabling the destruction callback for destroyed particles.
     * This function is locked during callbacks.
     *
     * @param The particle group to destroy.
     * @warning This function is locked during callbacks.
     */
    fun destroyParticlesInGroup(group: ParticleGroup?) {
        destroyParticlesInGroup(group, false)
    }

    /**
     * Get the world particle group list. With the returned group, use ParticleGroup::GetNext to get
     * the next group in the world list. A NULL group indicates the end of the list.
     *
     * @return the head of the world particle group list.
     */
    fun getParticleGroupList(): Array<ParticleGroup?>? {
        return m_particleSystem.particleGroupList
    }

    /**
     * Get the number of particle groups.
     *
     * @return
     */
    val particleGroupCount: Int
        get() = m_particleSystem.particleGroupCount

    /**
     * Get the number of particles.
     *
     * @return
     */
    val particleCount: Int
        get() = m_particleSystem.particleCount

    /**
     * Get the maximum number of particles.
     *
     * @return
     */
    var particleMaxCount: Int
        get() = m_particleSystem.particleMaxCount
        /**
         * Set the maximum number of particles.
         *
         * @param count
         */
        set(count) {
            m_particleSystem.particleMaxCount = count
        }

    /**
     * Change the particle density.
     *
     * @param density
     */
    var particleDensity: Float
        get() = m_particleSystem.particleDensity
        set(density) {
            m_particleSystem.particleDensity = density
        }

    /**
     * Change the particle gravity scale. Adjusts the effect of the global gravity vector on
     * particles. Default value is 1.0f.
     *
     * @param gravityScale
     */
    var particleGravityScale: Float
        get() = m_particleSystem.particleGravityScale
        set(gravityScale) {
            m_particleSystem.particleGravityScale = gravityScale
        }

    /**
     * Damping is used to reduce the velocity of particles. The damping parameter can be larger than
     * 1.0f but the damping effect becomes sensitive to the time step when the damping parameter is
     * large.
     *
     * @param damping
     */
    var particleDamping: Float
        get() = m_particleSystem.particleDamping
        set(damping) {
            m_particleSystem.particleDamping = damping
        }

    /**
     * Change the particle radius. You should set this only once, on world start. If you change the
     * radius during execution, existing particles may explode, shrink, or behave unexpectedly.
     *
     * @param radius
     */
    var particleRadius: Float
        get() = m_particleSystem.particleRadius
        set(radius) {
            m_particleSystem.particleRadius = radius
        }

    /**
     * Get the particle data. @return the pointer to the head of the particle data.
     *
     * @return
     */
    fun getParticleFlagsBuffer(): IntArray? {
        return m_particleSystem.particleFlagsBuffer
    }

    fun getParticlePositionBuffer(): Array<Vec2>? {
        return m_particleSystem.particlePositionBuffer
    }

    fun getParticleVelocityBuffer(): Array<Vec2>? {
        return m_particleSystem.particleVelocityBuffer
    }

    fun getParticleColorBuffer(): Array<ParticleColor?>? {
        return m_particleSystem.particleColorBuffer
    }

    fun getParticleGroupBuffer(): Array<ParticleGroup?>? {
        return m_particleSystem.particleGroupBuffer
    }

    fun getParticleUserDataBuffer(): Array<Any?>? {
        return m_particleSystem.particleUserDataBuffer
    }

    /**
     * Set a buffer for particle data.
     *
     * @param buffer is a pointer to a block of memory.
     * @param size is the number of values in the block.
     */
    fun setParticleFlagsBuffer(buffer: IntArray?, capacity: Int) {
        m_particleSystem.setParticleFlagsBuffer(buffer, capacity)
    }

    fun setParticlePositionBuffer(buffer: Array<Vec2>?, capacity: Int) {
        m_particleSystem.setParticlePositionBuffer(buffer, capacity)
    }

    fun setParticleVelocityBuffer(buffer: Array<Vec2>?, capacity: Int) {
        m_particleSystem.setParticleVelocityBuffer(buffer, capacity)
    }

    fun setParticleColorBuffer(buffer: Array<ParticleColor?>?, capacity: Int) {
        m_particleSystem.setParticleColorBuffer(buffer, capacity)
    }

    fun setParticleUserDataBuffer(buffer: Array<Any?>?, capacity: Int) {
        m_particleSystem.setParticleUserDataBuffer(buffer, capacity)
    }

    /**
     * Get contacts between particles
     *
     * @return
     */
    val particleContacts: Array<ParticleContact?>
        get() = m_particleSystem.m_contactBuffer
    val particleContactCount: Int
        get() = m_particleSystem.m_contactCount

    /**
     * Get contacts between particles and bodies
     *
     * @return
     */
    val particleBodyContacts: Array<ParticleBodyContact>
        get() = m_particleSystem.m_bodyContactBuffer
    val particleBodyContactCount: Int
        get() = m_particleSystem.m_bodyContactCount

    /**
     * Compute the kinetic energy that can be lost by damping force
     *
     * @return
     */
    fun computeParticleCollisionEnergy(): Float {
        return m_particleSystem.computeParticleCollisionEnergy()
    }

    companion object {
        const val WORLD_POOL_SIZE = 100
        const val WORLD_POOL_CONTAINER_SIZE = 10
        const val NEW_FIXTURE = 0x0001
        const val LOCKED = 0x0002
        const val CLEAR_FORCES = 0x0004

        // NOTE this corresponds to the liquid test, so the debugdraw can draw
        // the liquid particles correctly. They should be the same.
        private val LIQUID_INT = 1234598372
    }

    init {
        m_bodyList = null
        m_jointList = null
        m_bodyCount = 0
        m_jointCount = 0
        isWarmStarting = true
        isContinuousPhysics = true
        isSubStepping = false
        m_stepComplete = true
        isSleepingAllowed = true
        m_gravity.set(gravity)
        m_flags = CLEAR_FORCES
        m_inv_dt0 = 0f
        m_contactManager = ContactManager(this, broadPhase)
        m_profile = Profile()
        m_particleSystem = ParticleSystem(this)
        initializeRegisters()
    }
}

internal class WorldQueryWrapper : TreeCallback {
    override fun treeCallback(nodeId: Int): Boolean {
        val proxy = broadPhase!!.getUserData(nodeId) as FixtureProxy
        return callback!!.reportFixture(proxy.fixture)
    }

    var broadPhase: BroadPhase? = null
    var callback: QueryCallback? = null
}

internal class WorldRayCastWrapper : TreeRayCastCallback {
    // djm pooling
    private val output = RayCastOutput()
    private val temp = Vec2()
    private val point = Vec2()
    override fun raycastCallback(input: RayCastInput, nodeId: Int): Float {
        val userData = broadPhase!!.getUserData(nodeId)
        val proxy = userData as FixtureProxy
        val fixture = proxy.fixture
        val index = proxy.childIndex
        val hit = fixture.raycast(output, input, index)
        if (hit) {
            val fraction = output.fraction
            // Vec2 point = (1.0f - fraction) * input.p1 + fraction * input.p2;
            temp.set(input.p2).mulLocal(fraction)
            point.set(input.p1).mulLocal(1 - fraction).addLocal(temp)
            return callback!!.reportFixture(fixture, point, output.normal, fraction)
        }
        return input.maxFraction
    }

    var broadPhase: BroadPhase? = null
    var callback: RayCastCallback? = null
}
