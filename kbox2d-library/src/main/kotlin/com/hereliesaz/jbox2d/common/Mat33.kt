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
 * A 3-by-3 matrix. Stored in column-major order.
 *
 * @author Daniel Murphy
 */
data class Mat33(
    @JvmField val ex: Vec3 = Vec3(),
    @JvmField val ey: Vec3 = Vec3(),
    @JvmField val ez: Vec3 = Vec3()
) : Serializable {

    /**
     * Creates a matrix from 9 floats.
     *
     * @param exx the x-component of the first column
     * @param exy the y-component of the first column
     * @param exz the z-component of the first column
     * @param eyx the x-component of the second column
     * @param eyy the y-component of the second column
     * @param eyz the z-component of the second column
     * @param ezx the x-component of the third column
     * @param ezy the y-component of the third column
     * @param ezz the z-component of the third column
     */
    constructor(
        exx: Float, exy: Float, exz: Float,
        eyx: Float, eyy: Float, eyz: Float,
        ezx: Float, ezy: Float, ezz: Float
    ) : this(Vec3(exx, exy, exz), Vec3(eyx, eyy, eyz), Vec3(ezx, ezy, ezz))

    /**
     * Sets this matrix to the zero matrix.
     */
    fun setZero() {
        ex.setZero()
        ey.setZero()
        ez.setZero()
    }

    /**
     * Sets this matrix from 9 floats.
     *
     * @param exx the x-component of the first column
     * @param exy the y-component of the first column
     * @param exz the z-component of the first column
     * @param eyx the x-component of the second column
     * @param eyy the y-component of the second column
     * @param eyz the z-component of the second column
     * @param ezx the x-component of the third column
     * @param ezy the y-component of the third column
     * @param ezz the z-component of the third column
     */
    fun set(
        exx: Float, exy: Float, exz: Float,
        eyx: Float, eyy: Float, eyz: Float,
        ezx: Float, ezy: Float, ezz: Float
    ) {
        ex.x = exx
        ex.y = exy
        ex.z = exz
        ey.x = eyx
        ey.y = eyy
        ey.z = eyz
        ez.x = ezx
        ez.y = ezy
        ez.z = ezz
    }

    /**
     * Sets this matrix to be a copy of another matrix.
     *
     * @param mat the matrix to copy from
     */
    fun set(mat: Mat33) {
        ex.set(mat.ex)
        ey.set(mat.ey)
        ez.set(mat.ez)
    }

    /**
     * Sets this matrix to the identity matrix.
     */
    fun setIdentity() {
        ex.x = 1f
        ex.y = 0f
        ex.z = 0f
        ey.x = 0f
        ey.y = 1f
        ey.z = 0f
        ez.x = 0f
        ez.y = 0f
        ez.z = 1f
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than
     * computing the inverse in one-shot cases.
     *
     * @param b the right-hand side vector
     * @return the solution vector x
     */
    fun solve22(b: Vec2): Vec2 {
        val x = Vec2()
        solve22ToOut(b, x)
        return x
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than
     * computing the inverse in one-shot cases.
     *
     * @param b the right-hand side vector
     * @param out the vector to store the solution in
     */
    fun solve22ToOut(b: Vec2, out: Vec2) {
        val a11 = ex.x
        val a12 = ey.x
        val a21 = ex.y
        val a22 = ey.y
        var det = a11 * a22 - a12 * a21
        if (det != 0.0f) {
            det = 1.0f / det
        }
        out.x = det * (a22 * b.x - a12 * b.y)
        out.y = det * (a11 * b.y - a21 * b.x)
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than
     * computing the inverse in one-shot cases.
     *
     * @param b the right-hand side vector
     * @return the solution vector x
     */
    fun solve33(b: Vec3): Vec3 {
        val x = Vec3()
        solve33ToOut(b, x)
        return x
    }

    /**
     * Solve A * x = b, where b is a column vector. This is more efficient than
     * computing the inverse in one-shot cases.
     *
     * @param b the right-hand side vector
     * @param out the vector to store the solution in
     */
    fun solve33ToOut(b: Vec3, out: Vec3) {
        assert(b !== out)
        Vec3.crossToOutUnsafe(ey, ez, out)
        var det = Vec3.dot(ex, out)
        if (det != 0.0f) {
            det = 1.0f / det
        }
        Vec3.crossToOutUnsafe(ey, ez, out)
        val x = det * Vec3.dot(b, out)
        Vec3.crossToOutUnsafe(b, ez, out)
        val y = det * Vec3.dot(ex, out)
        Vec3.crossToOutUnsafe(ey, b, out)
        val z = det * Vec3.dot(ex, out)
        out.x = x
        out.y = y
        out.z = z
    }

    /**
     * Get the inverse of this matrix as a 2x2.
     * Returns the zero matrix if singular.
     *
     * @param M the matrix to store the result in
     */
    fun getInverse22(M: Mat33) {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        var det = a * d - b * c
        if (det != 0.0f) {
            det = 1.0f / det
        }
        M.ex.x = det * d
        M.ey.x = -det * b
        M.ex.z = 0.0f
        M.ex.y = -det * c
        M.ey.y = det * a
        M.ey.z = 0.0f
        M.ez.x = 0.0f
        M.ez.y = 0.0f
        M.ez.z = 0.0f
    }

    /**
     * Get the symmetric inverse of this matrix as a 3x3.
     * Returns the zero matrix if singular.
     *
     * @param M the matrix to store the result in
     */
    fun getSymInverse33(M: Mat33) {
        val bx = ey.y * ez.z - ey.z * ez.y
        val by = ey.z * ez.x - ey.x * ez.z
        val bz = ey.x * ez.y - ey.y * ez.x
        var det = ex.x * bx + ex.y * by + ex.z * bz
        if (det != 0.0f) {
            det = 1.0f / det
        }
        val a11 = ex.x
        val a12 = ey.x
        val a13 = ez.x
        val a22 = ey.y
        val a23 = ez.y
        val a33 = ez.z
        M.ex.x = det * (a22 * a33 - a23 * a23)
        M.ex.y = det * (a13 * a23 - a12 * a33)
        M.ex.z = det * (a12 * a23 - a13 * a22)
        M.ey.x = M.ex.y
        M.ey.y = det * (a11 * a33 - a13 * a13)
        M.ey.z = det * (a13 * a12 - a11 * a23)
        M.ez.x = M.ex.z
        M.ez.y = M.ey.z
        M.ez.z = det * (a11 * a22 - a12 * a12)
    }

    companion object {
        private const val serialVersionUID = 2L

        /**
         * The identity matrix.
         */
        @JvmField
        val IDENTITY = Mat33(Vec3(1f, 0f, 0f), Vec3(0f, 1f, 0f), Vec3(0f, 0f, 1f))

        /**
         * Multiplies a matrix by a vector.
         *
         * @param A the matrix
         * @param v the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun mul(A: Mat33, v: Vec3): Vec3 {
            return Vec3(
                v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x,
                v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y,
                v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z
            )
        }

        /**
         * Multiplies the 2x2 part of a matrix by a 2D vector.
         *
         * @param A the matrix
         * @param v the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun mul22(A: Mat33, v: Vec2): Vec2 {
            return Vec2(A.ex.x * v.x + A.ey.x * v.y, A.ex.y * v.x + A.ey.y * v.y)
        }

        /**
         * Multiplies the 2x2 part of a matrix by a 2D vector and stores the result in the output vector.
         *
         * @param A the matrix
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mul22ToOut(A: Mat33, v: Vec2, out: Vec2) {
            val tempX = A.ex.x * v.x + A.ey.x * v.y
            out.y = A.ex.y * v.x + A.ey.y * v.y
            out.x = tempX
        }

        /**
         * Multiplies the 2x2 part of a matrix by a 2D vector and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param A the matrix
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mul22ToOutUnsafe(A: Mat33, v: Vec2, out: Vec2) {
            assert(v !== out)
            out.y = A.ex.y * v.x + A.ey.y * v.y
            out.x = A.ex.x * v.x + A.ey.x * v.y
        }

        /**
         * Multiplies a matrix by a vector and stores the result in the output vector.
         *
         * @param A the matrix
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulToOut(A: Mat33, v: Vec3, out: Vec3) {
            val tempY = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y
            val tempZ = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z
            out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x
            out.y = tempY
            out.z = tempZ
        }

        /**
         * Multiplies a matrix by a vector and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param A the matrix
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulToOutUnsafe(A: Mat33, v: Vec3, out: Vec3) {
            assert(out !== v)
            out.x = v.x * A.ex.x + v.y * A.ey.x + v.z * A.ez.x
            out.y = v.x * A.ex.y + v.y * A.ey.y + v.z * A.ez.y
            out.z = v.x * A.ex.z + v.y * A.ey.z + v.z * A.ez.z
        }

        /**
         * Creates a matrix that represents a scaling transformation and stores it in the output matrix.
         *
         * @param scale the scaling factor
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun setScaleTransform(scale: Float, out: Mat33) {
            out.ex.x = scale
            out.ey.y = scale
        }
    }
}
