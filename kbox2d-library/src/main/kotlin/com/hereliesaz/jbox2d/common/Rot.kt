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
package com.hereliesaz.jbox2d.common

import java.io.Serializable

/**
 * Represents a rotation
 *
 * @author Daniel Murphy
 */
class Rot : Serializable {

    /**
     * sin
     */
    @JvmField
    var s: Float = 0f

    /**
     * cos
     */
    @JvmField
    var c: Float = 0f

    constructor() {
        setIdentity()
    }

    constructor(angle: Float) {
        set(angle)
    }

    val sin: Float
        get() = s

    override fun toString(): String {
        return "Rot(s:$s, c:$c)"
    }

    val cos: Float
        get() = c

    fun set(angle: Float): Rot {
        s = MathUtils.sin(angle)
        c = MathUtils.cos(angle)
        return this
    }

    fun set(other: Rot): Rot {
        s = other.s
        c = other.c
        return this
    }

    fun setIdentity(): Rot {
        s = 0f
        c = 1f
        return this
    }

    val angle: Float
        get() = MathUtils.atan2(s, c)

    fun getXAxis(xAxis: Vec2) {
        xAxis.set(c, s)
    }

    fun getYAxis(yAxis: Vec2) {
        yAxis.set(-s, c)
    }

    public fun clone(): Rot {
        val copy = Rot()
        copy.s = s
        copy.c = c
        return copy
    }

    companion object {
        private const val serialVersionUID = 1L

        @JvmStatic
        fun mul(q: Rot, r: Rot, out: Rot) {
            val tempC = q.c * r.c - q.s * r.s
            out.s = q.s * r.c + q.c * r.s
            out.c = tempC
        }

        /**
         * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L535-L546
         */
        @JvmStatic
        fun mulUnsafe(q: Rot, r: Rot, out: Rot) {
            assert(r !== out)
            assert(q !== out)
            out.s = q.s * r.c + q.c * r.s
            out.c = q.c * r.c - q.s * r.s
        }

        @JvmStatic
        fun mulTrans(q: Rot, r: Rot, out: Rot) {
            val tempC = q.c * r.c + q.s * r.s
            out.s = q.c * r.s - q.s * r.c
            out.c = tempC
        }

        /**
         * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L548-L559
         */
        @JvmStatic
        fun mulTransUnsafe(q: Rot, r: Rot, out: Rot) {
            out.s = q.c * r.s - q.s * r.c
            out.c = q.c * r.c + q.s * r.s
        }

        @JvmStatic
        fun mulToOut(q: Rot, v: Vec2, out: Vec2) {
            val tempY = q.s * v.x + q.c * v.y
            out.x = q.c * v.x - q.s * v.y
            out.y = tempY
        }

        @JvmStatic
        fun mulToOutUnsafe(q: Rot, v: Vec2, out: Vec2) {
            out.x = q.c * v.x - q.s * v.y
            out.y = q.s * v.x + q.c * v.y
        }

        @JvmStatic
        fun mulTrans(q: Rot, v: Vec2, out: Vec2) {
            val tempY = -q.s * v.x + q.c * v.y
            out.x = q.c * v.x + q.s * v.y
            out.y = tempY
        }

        @JvmStatic
        fun mulTransUnsafe(q: Rot, v: Vec2, out: Vec2) {
            out.x = q.c * v.x + q.s * v.y
            out.y = -q.s * v.x + q.c * v.y
        }
    }
}
