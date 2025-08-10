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
package com.hereliesaz.kbox2d.common

import java.io.Serializable

/**
 * A 2D column vector.
 *
 * @author Daniel Murphy
 *
 * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L40-L129
 */
data class Vec2(var x: Float = 0f, var y: Float = 0f) : Serializable {

    companion object {
        private const val serialVersionUID = 1L

        fun abs(a: Vec2): Vec2 {
            return Vec2(MathUtils.abs(a.x), MathUtils.abs(a.y))
        }

        fun absToOut(a: Vec2, out: Vec2) {
            out.x = MathUtils.abs(a.x)
            out.y = MathUtils.abs(a.y)
        }

        fun dot(a: Vec2, b: Vec2): Float {
            return a.x * b.x + a.y * b.y
        }

        fun cross(a: Vec2, b: Vec2): Float {
            return a.x * b.y - a.y * b.x
        }

        fun cross(a: Vec2, s: Float): Vec2 {
            return Vec2(s * a.y, -s * a.x)
        }

        fun crossToOut(a: Vec2, s: Float, out: Vec2) {
            val tempY = -s * a.x
            out.x = s * a.y
            out.y = tempY
        }

        fun crossToOutUnsafe(a: Vec2, s: Float, out: Vec2) {
            assert(out !== a)
            out.x = s * a.y
            out.y = -s * a.x
        }

        fun cross(s: Float, a: Vec2): Vec2 {
            return Vec2(-s * a.y, s * a.x)
        }

        fun crossToOut(s: Float, a: Vec2, out: Vec2) {
            val tempY = s * a.x
            out.x = -s * a.y
            out.y = tempY
        }

        fun crossToOutUnsafe(s: Float, a: Vec2, out: Vec2) {
            assert(out !== a)
            out.x = -s * a.y
            out.y = s * a.x
        }

        fun negateToOut(a: Vec2, out: Vec2) {
            out.x = -a.x
            out.y = -a.y
        }

        fun min(a: Vec2, b: Vec2): Vec2 {
            return Vec2(Math.min(a.x, b.x), Math.min(a.y, b.y))
        }

        fun max(a: Vec2, b: Vec2): Vec2 {
            return Vec2(Math.max(a.x, b.x), Math.max(a.y, b.y))
        }

        fun minToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = Math.min(a.x, b.x)
            out.y = Math.min(a.y, b.y)
        }

        fun maxToOut(a: Vec2, b: Vec2, out: Vec2) {
            out.x = Math.max(a.x, b.x)
            out.y = Math.max(a.y, b.y)
        }
    }

    constructor(toCopy: Vec2) : this(toCopy.x, toCopy.y)

    /**
     * Zero out this vector.
     */
    fun setZero() {
        x = 0.0f
        y = 0.0f
    }

    /**
     * Set the vector component-wise.
     */
    fun set(x: Float, y: Float): Vec2 {
        this.x = x
        this.y = y
        return this
    }

    /**
     * Set this vector to another vector.
     */
    fun set(v: Vec2): Vec2 {
        this.x = v.x
        this.y = v.y
        return this
    }

    /**
     * Return the sum of this vector and another; does not alter either one.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L149-L153
     */
    fun add(v: Vec2): Vec2 {
        return Vec2(x + v.x, y + v.y)
    }

    /**
     * Return the difference of this vector and another; does not alter either
     * one.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L155-L159
     */
    fun sub(v: Vec2): Vec2 {
        return Vec2(x - v.x, y - v.y)
    }

    /**
     * Return this vector multiplied by a scalar; does not alter this vector.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L161-L165
     */
    fun mul(a: Float): Vec2 {
        return Vec2(x * a, y * a)
    }

    /**
     * Return the negation of this vector; does not alter this vector.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L146-L147
     */
    fun negate(): Vec2 {
        return Vec2(-x, -y)
    }

    /**
     * Flip the vector and return it - alters this vector.
     */
    fun negateLocal(): Vec2 {
        x = -x
        y = -y
        return this
    }

    /**
     * Add another vector to this one and returns result - alters this vector.
     */
    fun addLocal(v: Vec2): Vec2 {
        x += v.x
        y += v.y
        return this
    }

    /**
     * Adds values to this vector and returns result - alters this vector.
     */
    fun addLocal(x: Float, y: Float): Vec2 {
        this.x += x
        this.y += y
        return this
    }

    /**
     * Subtract another vector from this one and return result - alters this
     * vector.
     */
    fun subLocal(v: Vec2): Vec2 {
        x -= v.x
        y -= v.y
        return this
    }

    /**
     * Multiply this vector by a number and return result - alters this vector.
     */
    fun mulLocal(a: Float): Vec2 {
        x *= a
        y *= a
        return this
    }

    /**
     * Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
     */
    fun skew(): Vec2 {
        return Vec2(-y, x)
    }

    /**
     * Get the skew vector such that dot(skew_vec, other) == cross(vec, other)
     */
    fun skew(out: Vec2) {
        out.x = -y
        out.y = x
    }

    /**
     * Get the length of this vector (the norm). Return the length of this
     * vector.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L88-L92
     */
    fun length(): Float {
        return MathUtils.sqrt(x * x + y * y)
    }

    /**
     * Get the length squared. For performance, use this instead of
     * [Vec2.length]. Return the squared length of this vector.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L94-L99
     */
    fun lengthSquared(): Float {
        return x * x + y * y
    }

    /**
     * Convert this vector into a unit vector. Returns the length. Normalize
     * this vector and return the length before normalization. Alters this
     * vector.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L101-L114
     */
    fun normalize(): Float {
        val length = length()
        if (length < Settings.EPSILON) {
            return 0f
        }
        val invLength = 1.0f / length
        x *= invLength
        y *= invLength
        return length
    }

    /**
     * Does this vector contain finite coordinates? True if the vector
     * represents a pair of valid, non-infinite floating point numbers.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L116-L120
     */
    fun isValid(): Boolean {
        return !java.lang.Float.isNaN(x) && !java.lang.Float.isInfinite(x) && !java.lang.Float.isNaN(y) && !java.lang.Float.isInfinite(y)
    }

    /**
     * Return a new vector that has positive components.
     */
    fun abs(): Vec2 {
        return Vec2(MathUtils.abs(x), MathUtils.abs(y))
    }

    fun absLocal() {
        x = MathUtils.abs(x)
        y = MathUtils.abs(y)
    }
}
