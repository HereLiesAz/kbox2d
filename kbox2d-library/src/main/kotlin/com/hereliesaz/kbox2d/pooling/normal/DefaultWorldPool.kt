/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kbox2d.pooling.normal

import com.hereliesaz.kbox2d.collision.AABB
import com.hereliesaz.kbox2d.collision.Collision
import com.hereliesaz.kbox2d.collision.Distance
import com.hereliesaz.kbox2d.collision.TimeOfImpact
import com.hereliesaz.kbox2d.common.*
import com.hereliesaz.kbox2d.dynamics.contacts.*
import com.hereliesaz.kbox2d.pooling.DynamicStack
import com.hereliesaz.kbox2d.pooling.WorldPool
import java.util.HashMap

/**
 * Provides object pooling for all objects used in the engine. Objects retrieved
 * from here should only be used temporarily, and then pushed back (except *
 * arrays).
 *
 * @author Daniel Murphy
 */
class DefaultWorldPool(argSize: Int, argContainerSize: Int) : WorldPool {
    private val vecs: OrderedStack<Vec2>
    private val vec3s: OrderedStack<Vec3>
    private val mats: OrderedStack<Mat22>
    private val mat33s: OrderedStack<Mat33>
    private val aabbs: OrderedStack<AABB>
    private val rots: OrderedStack<Rot>
    private val afloats = HashMap<Int, FloatArray>()
    private val aints = HashMap<Int, IntArray>()
    private val avecs = HashMap<Int, Array<Vec2>>()
    private val world: WorldPool = this
    private val pcstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return PolygonContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    private val ccstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return CircleContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    private val cpstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return PolygonAndCircleContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    private val ecstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return EdgeAndCircleContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    private val epstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return EdgeAndPolygonContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    private val chcstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return ChainAndCircleContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    private val chpstack: MutableStack<Contact> =
        object : MutableStack<Contact>(Settings.CONTACT_STACK_INIT_SIZE) {
            override fun newInstance(): Contact {
                return ChainAndPolygonContact(world)
            }

            override fun newArray(size: Int): Array<Contact?> {
                return arrayOfNulls(size)
            }
        }
    override val collision: Collision
    override val timeOfImpact: TimeOfImpact
    override val distance: Distance

    init {
        vecs = object : OrderedStack<Vec2>(argSize, argContainerSize) {
            override fun newInstance(): Vec2 {
                return Vec2()
            }
        }
        vec3s = object : OrderedStack<Vec3>(argSize, argContainerSize) {
            override fun newInstance(): Vec3 {
                return Vec3()
            }
        }
        mats = object : OrderedStack<Mat22>(argSize, argContainerSize) {
            override fun newInstance(): Mat22 {
                return Mat22()
            }
        }
        aabbs = object : OrderedStack<AABB>(argSize, argContainerSize) {
            override fun newInstance(): AABB {
                return AABB()
            }
        }
        rots = object : OrderedStack<Rot>(argSize, argContainerSize) {
            override fun newInstance(): Rot {
                return Rot()
            }
        }
        mat33s = object : OrderedStack<Mat33>(argSize, argContainerSize) {
            override fun newInstance(): Mat33 {
                return Mat33()
            }
        }
        dist = Distance()
        collision = Collision(this)
        toi = TimeOfImpact(this)
    }

    override val polyContactStack: DynamicStack<Contact>
        get() = pcstack
    override val circleContactStack: DynamicStack<Contact>
        get() = ccstack
    override val polyCircleContactStack: DynamicStack<Contact>
        get() = cpstack
    override val edgeCircleContactStack: DynamicStack<Contact>
        get() = ecstack
    override val edgePolyContactStack: DynamicStack<Contact>
        get() = epstack
    override val chainCircleContactStack: DynamicStack<Contact>
        get() = chcstack
    override val chainPolyContactStack: DynamicStack<Contact>
        get() = chpstack

    override fun popVec2(): Vec2 {
        return vecs.pop()
    }

    override fun popVec2(argNum: Int): Array<Vec2> {
        return vecs.pop(argNum)
    }

    override fun pushVec2(argNum: Int) {
        vecs.push(argNum)
    }

    override fun popVec3(): Vec3 {
        return vec3s.pop()
    }

    override fun popVec3(argNum: Int): Array<Vec3> {
        return vec3s.pop(argNum)
    }

    override fun pushVec3(argNum: Int) {
        vec3s.push(argNum)
    }

    override fun popMat22(): Mat22 {
        return mats.pop()
    }

    override fun popMat22(argNum: Int): Array<Mat22> {
        return mats.pop(argNum)
    }

    override fun pushMat22(argNum: Int) {
        mats.push(argNum)
    }

    override fun popMat33(): Mat33 {
        return mat33s.pop()
    }

    override fun pushMat33(argNum: Int) {
        mat33s.push(argNum)
    }

    override fun popAABB(): AABB {
        return aabbs.pop()
    }

    override fun popAABB(argNum: Int): Array<AABB> {
        return aabbs.pop(argNum)
    }

    override fun pushAABB(argNum: Int) {
        aabbs.push(argNum)
    }

    override fun popRot(): Rot {
        return rots.pop()
    }

    override fun pushRot(num: Int) {
        rots.push(num)
    }

    override fun getFloatArray(argLength: Int): FloatArray {
        if (!afloats.containsKey(argLength)) {
            afloats[argLength] = FloatArray(argLength)
        }
        assert(afloats[argLength]!!.size == argLength) { "Array not built with correct length" }
        return afloats[argLength]!!
    }

    override fun getIntArray(argLength: Int): IntArray {
        if (!aints.containsKey(argLength)) {
            aints[argLength] = IntArray(argLength)
        }
        assert(aints[argLength]!!.size == argLength) { "Array not built with correct length" }
        return aints[argLength]!!
    }

    override fun getVec2Array(argLength: Int): Array<Vec2> {
        if (!avecs.containsKey(argLength)) {
            val ray = Array(argLength) { Vec2() }
            for (i in 0 until argLength) {
                ray[i] = Vec2()
            }
            avecs[argLength] = ray
        }
        assert(avecs[argLength]!!.size == argLength) { "Array not built with correct length" }
        return avecs[argLength]!!
    }
}
