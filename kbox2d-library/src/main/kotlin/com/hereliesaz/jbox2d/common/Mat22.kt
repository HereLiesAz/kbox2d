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
 * A 2-by-2 matrix. Stored in column-major order.
 *
 * @author Daniel Murphy
 */
data class Mat22(
    @JvmField val ex: Vec2 = Vec2(),
    @JvmField val ey: Vec2 = Vec2()
) : Serializable {

    /**
     * Creates a matrix from four floats.
     *
     * @param exx the x-component of the first column
     * @param col2x the x-component of the second column
     * @param exy the y-component of the first column
     * @param col2y the y-component of the second column
     */
    constructor(exx: Float, col2x: Float, exy: Float, col2y: Float) : this(Vec2(exx, exy), Vec2(col2x, col2y))

    /**
     * Sets this matrix as a copy of another matrix.
     *
     * @param m the matrix to copy
     * @return this matrix for chaining
     */
    fun set(m: Mat22): Mat22 {
        ex.x = m.ex.x
        ex.y = m.ex.y
        ey.x = m.ey.x
        ey.y = m.ey.y
        return this
    }

    /**
     * Sets this matrix from four floats.
     *
     * @param exx the x-component of the first column
     * @param col2x the x-component of the second column
     * @param exy the y-component of the first column
     * @param col2y the y-component of the second column
     * @return this matrix for chaining
     */
    fun set(exx: Float, col2x: Float, exy: Float, col2y: Float): Mat22 {
        ex.x = exx
        ex.y = exy
        ey.x = col2x
        ey.y = col2y
        return this
    }

    /**
     * Sets this matrix to represent a rotation.
     *
     * @param angle the rotation angle in radians
     */
    fun set(angle: Float) {
        val c = MathUtils.cos(angle)
        val s = MathUtils.sin(angle)
        ex.x = c
        ey.x = -s
        ex.y = s
        ey.y = c
    }

    /**
     * Sets this matrix to the identity matrix.
     */
    fun setIdentity() {
        ex.x = 1.0f
        ey.x = 0.0f
        ex.y = 0.0f
        ey.y = 1.0f
    }

    /**
     * Sets this matrix to the zero matrix.
     */
    fun setZero() {
        ex.x = 0.0f
        ey.x = 0.0f
        ex.y = 0.0f
        ey.y = 0.0f
    }

    /**
     * Extracts the angle from this matrix (assumed to be a rotation matrix).
     *
     * @return the angle in radians
     */
    val angle: Float
        get() = MathUtils.atan2(ex.y, ex.x)

    /**
     * Inverts this matrix.
     * This operation modifies this matrix.
     *
     * @return this matrix for chaining
     */
    fun invertLocal(): Mat22 {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        var det = a * d - b * c
        if (det != 0f) {
            det = 1.0f / det
        }
        ex.x = det * d
        ey.x = -det * b
        ex.y = -det * c
        ey.y = det * a
        return this
    }

    /**
     * Computes the inverse of this matrix and stores it in the given output matrix.
     *
     * @param out the matrix to store the result in
     */
    fun invertToOut(out: Mat22) {
        val a = ex.x
        val b = ey.x
        val c = ex.y
        val d = ey.y
        var det = a * d - b * c
        // b2Assert(det != 0.0f);
        det = 1.0f / det
        out.ex.x = det * d
        out.ey.x = -det * b
        out.ex.y = -det * c
        out.ey.y = det * a
    }

    /**
     * Sets the components of this matrix to their absolute values.
     */
    fun absLocal() {
        ex.absLocal()
        ey.absLocal()
    }

    /**
     * Multiplies this matrix by another matrix.
     * This operation modifies this matrix.
     *
     * @param R the matrix to multiply by
     * @return this matrix for chaining
     */
    fun mulLocal(R: Mat22): Mat22 {
        mulToOut(R, this)
        return this
    }

    /**
     * Multiplies this matrix by another matrix and stores the result in the output matrix.
     *
     * @param R the matrix to multiply by
     * @param out the matrix to store the result in
     */
    fun mulToOut(R: Mat22, out: Mat22) {
        val tempY1 = this.ex.y * R.ex.x + this.ey.y * R.ex.y
        out.ex.x = this.ex.x * R.ex.x + this.ey.x * R.ex.y
        out.ex.y = tempY1
        val tempY2 = this.ex.y * R.ey.x + this.ey.y * R.ey.y
        out.ey.x = this.ex.x * R.ey.x + this.ey.x * R.ey.y
        out.ey.y = tempY2
    }

    /**
     * Multiplies this matrix by another matrix and stores the result in the output matrix.
     * This is an unsafe version that assumes the output matrix is not the same as the input matrices.
     *
     * @param R the matrix to multiply by
     * @param out the matrix to store the result in
     */
    fun mulToOutUnsafe(R: Mat22, out: Mat22) {
        assert(out !== R)
        assert(out !== this)
        out.ex.x = this.ex.x * R.ex.x + this.ey.x * R.ex.y
        out.ex.y = this.ex.y * R.ex.x + this.ey.y * R.ex.y
        out.ey.x = this.ex.x * R.ey.x + this.ey.x * R.ey.y
        out.ey.y = this.ex.y * R.ey.x + this.ey.y * R.ey.y
    }

    /**
     * Multiplies the transpose of this matrix by another matrix.
     * This operation modifies this matrix.
     *
     * @param B the matrix to multiply by
     * @return this matrix for chaining
     */
    fun mulTransLocal(B: Mat22): Mat22 {
        mulTransToOut(B, this)
        return this
    }

    /**
     * Multiplies the transpose of this matrix by another matrix and stores the result in the output matrix.
     *
     * @param B the matrix to multiply by
     * @param out the matrix to store the result in
     */
    fun mulTransToOut(B: Mat22, out: Mat22) {
        val x1 = this.ex.x * B.ex.x + this.ex.y * B.ex.y
        val y1 = this.ey.x * B.ex.x + this.ey.y * B.ex.y
        val x2 = this.ex.x * B.ey.x + this.ex.y * B.ey.y
        val y2 = this.ey.x * B.ey.x + this.ey.y * B.ey.y
        out.ex.x = x1
        out.ey.x = x2
        out.ex.y = y1
        out.ey.y = y2
    }

    /**
     * Multiplies the transpose of this matrix by another matrix and stores the result in the output matrix.
     * This is an unsafe version that assumes the output matrix is not the same as the input matrices.
     *
     * @param B the matrix to multiply by
     * @param out the matrix to store the result in
     */
    fun mulTransToOutUnsafe(B: Mat22, out: Mat22) {
        assert(B !== out)
        assert(this !== out)
        out.ex.x = this.ex.x * B.ex.x + this.ex.y * B.ex.y
        out.ey.x = this.ex.x * B.ey.x + this.ex.y * B.ey.y
        out.ex.y = this.ey.x * B.ex.x + this.ey.y * B.ex.y
        out.ey.y = this.ey.x * B.ey.x + this.ey.y * B.ey.y
    }

    /**
     * Adds another matrix to this one.
     * This operation modifies this matrix.
     *
     * @param B the matrix to add
     * @return this matrix for chaining
     */
    fun addLocal(B: Mat22): Mat22 {
        ex.x += B.ex.x
        ex.y += B.ex.y
        ey.x += B.ey.x
        ey.y += B.ey.y
        return this
    }

    /**
     * Solve A * x = b where A = this matrix.
     *
     * @param b the right-hand side vector
     * @return the solution vector x
     */
    fun solve(b: Vec2): Vec2 {
        val a11 = ex.x
        val a12 = ey.x
        val a21 = ex.y
        val a22 = ey.y
        var det = a11 * a22 - a12 * a21
        if (det != 0.0f) {
            det = 1.0f / det
        }
        return Vec2(det * (a22 * b.x - a12 * b.y), det * (a11 * b.y - a21 * b.x))
    }

    /**
     * Solve A * x = b where A = this matrix, and stores the result in the output vector.
     *
     * @param b the right-hand side vector
     * @param out the vector to store the solution in
     */
    fun solveToOut(b: Vec2, out: Vec2) {
        val a11 = ex.x
        val a12 = ey.x
        val a21 = ex.y
        val a22 = ey.y
        var det = a11 * a22 - a12 * a21
        if (det != 0.0f) {
            det = 1.0f / det
        }
        val tempY = det * (a11 * b.y - a21 * b.x)
        out.x = det * (a22 * b.x - a12 * b.y)
        out.y = tempY
    }

    override fun toString(): String {
        return "[$ex, $ey]"
    }

    companion object {
        private const val serialVersionUID = 2L

        /**
         * Returns a new matrix with the absolute values of the components of the given matrix.
         *
         * @param R the matrix to take the absolute values of
         * @return a new matrix with the absolute values
         */
        @JvmStatic
        fun abs(R: Mat22): Mat22 {
            return Mat22(Vec2.abs(R.ex), Vec2.abs(R.ey))
        }

        /**
         * Sets the output matrix to the absolute values of the components of the given matrix.
         *
         * @param R the matrix to take the absolute values of
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun absToOut(R: Mat22, out: Mat22) {
            out.ex.x = MathUtils.abs(R.ex.x)
            out.ex.y = MathUtils.abs(R.ex.y)
            out.ey.x = MathUtils.abs(R.ey.x)
            out.ey.y = MathUtils.abs(R.ey.y)
        }

        /**
         * Multiplies a matrix by a vector.
         *
         * @param R the matrix
         * @param v the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun mul(R: Mat22, v: Vec2): Vec2 {
            return Vec2(R.ex.x * v.x + R.ey.x * v.y, R.ex.y * v.x + R.ey.y * v.y)
        }

        /**
         * Multiplies a matrix by a vector and stores the result in the output vector.
         *
         * @param R the matrix
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulToOut(R: Mat22, v: Vec2, out: Vec2) {
            val tempY = R.ex.y * v.x + R.ey.y * v.y
            out.x = R.ex.x * v.x + R.ey.x * v.y
            out.y = tempY
        }

        /**
         * Multiplies a matrix by a vector and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param R the matrix
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulToOutUnsafe(R: Mat22, v: Vec2, out: Vec2) {
            assert(v !== out)
            out.x = R.ex.x * v.x + R.ey.x * v.y
            out.y = R.ex.y * v.x + R.ey.y * v.y
        }

        /**
         * Multiplies two matrices.
         *
         * @param A the first matrix
         * @param B the second matrix
         * @return a new matrix containing the result
         */
        @JvmStatic
        fun mul(A: Mat22, B: Mat22): Mat22 {
            val C = Mat22()
            C.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y
            C.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y
            C.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y
            C.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y
            return C
        }

        /**
         * Multiplies two matrices and stores the result in the output matrix.
         *
         * @param A the first matrix
         * @param B the second matrix
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun mulToOut(A: Mat22, B: Mat22, out: Mat22) {
            val tempY1 = A.ex.y * B.ex.x + A.ey.y * B.ex.y
            val tempX1 = A.ex.x * B.ex.x + A.ey.x * B.ex.y
            val tempY2 = A.ex.y * B.ey.x + A.ey.y * B.ey.y
            val tempX2 = A.ex.x * B.ey.x + A.ey.x * B.ey.y
            out.ex.x = tempX1
            out.ex.y = tempY1
            out.ey.x = tempX2
            out.ey.y = tempY2
        }

        /**
         * Multiplies two matrices and stores the result in the output matrix.
         * This is an unsafe version that assumes the output matrix is not the same as the input matrices.
         *
         * @param A the first matrix
         * @param B the second matrix
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun mulToOutUnsafe(A: Mat22, B: Mat22, out: Mat22) {
            assert(out !== A)
            assert(out !== B)
            out.ex.x = A.ex.x * B.ex.x + A.ey.x * B.ex.y
            out.ex.y = A.ex.y * B.ex.x + A.ey.y * B.ex.y
            out.ey.x = A.ex.x * B.ey.x + A.ey.x * B.ey.y
            out.ey.y = A.ex.y * B.ey.x + A.ey.y * B.ey.y
        }

        /**
         * Multiplies the transpose of a matrix by a vector.
         *
         * @param R the matrix to transpose
         * @param v the vector
         * @return a new vector containing the result
         */
        @JvmStatic
        fun mulTrans(R: Mat22, v: Vec2): Vec2 {
            return Vec2((v.x * R.ex.x + v.y * R.ex.y), (v.x * R.ey.x + v.y * R.ey.y))
        }

        /**
         * Multiplies the transpose of a matrix by a vector and stores the result in the output vector.
         *
         * @param R the matrix to transpose
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulTransToOut(R: Mat22, v: Vec2, out: Vec2) {
            val outX = v.x * R.ex.x + v.y * R.ex.y
            out.y = v.x * R.ey.x + v.y * R.ey.y
            out.x = outX
        }

        /**
         * Multiplies the transpose of a matrix by a vector and stores the result in the output vector.
         * This is an unsafe version that assumes the output vector is not the same as the input vector.
         *
         * @param R the matrix to transpose
         * @param v the vector
         * @param out the vector to store the result in
         */
        @JvmStatic
        fun mulTransToOutUnsafe(R: Mat22, v: Vec2, out: Vec2) {
            assert(out !== v)
            out.y = v.x * R.ey.x + v.y * R.ey.y
            out.x = v.x * R.ex.x + v.y * R.ex.y
        }

        /**
         * Multiplies the transpose of a matrix by another matrix.
         *
         * @param A the first matrix (will be transposed)
         * @param B the second matrix
         * @return a new matrix containing the result
         */
        @JvmStatic
        fun mulTrans(A: Mat22, B: Mat22): Mat22 {
            val C = Mat22()
            C.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y
            C.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y
            C.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y
            C.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y
            return C
        }

        /**
         * Multiplies the transpose of a matrix by another matrix and stores the result in the output matrix.
         *
         * @param A the first matrix (will be transposed)
         * @param B the second matrix
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun mulTransToOut(A: Mat22, B: Mat22, out: Mat22) {
            val x1 = A.ex.x * B.ex.x + A.ex.y * B.ex.y
            val y1 = A.ey.x * B.ex.x + A.ey.y * B.ex.y
            val x2 = A.ex.x * B.ey.x + A.ex.y * B.ey.y
            val y2 = A.ey.x * B.ey.x + A.ey.y * B.ey.y
            out.ex.x = x1
            out.ex.y = y1
            out.ey.x = x2
            out.ey.y = y2
        }

        /**
         * Multiplies the transpose of a matrix by another matrix and stores the result in the output matrix.
         * This is an unsafe version that assumes the output matrix is not the same as the input matrices.
         *
         * @param A the first matrix (will be transposed)
         * @param B the second matrix
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun mulTransToOutUnsafe(A: Mat22, B: Mat22, out: Mat22) {
            assert(A !== out)
            assert(B !== out)
            out.ex.x = A.ex.x * B.ex.x + A.ex.y * B.ex.y
            out.ex.y = A.ey.x * B.ex.x + A.ey.y * B.ex.y
            out.ey.x = A.ex.x * B.ey.x + A.ex.y * B.ey.y
            out.ey.y = A.ey.x * B.ey.x + A.ey.y * B.ey.y
        }

        /**
         * Creates a matrix that represents a rotation.
         *
         * @param angle the angle of rotation in radians
         * @return a new matrix representing the rotation
         */
        @JvmStatic
        fun createRotationalTransform(angle: Float): Mat22 {
            val mat = Mat22()
            val c = MathUtils.cos(angle)
            val s = MathUtils.sin(angle)
            mat.ex.x = c
            mat.ey.x = -s
            mat.ex.y = s
            mat.ey.y = c
            return mat
        }

        /**
         * Creates a matrix that represents a rotation and stores it in the output matrix.
         *
         * @param angle the angle of rotation in radians
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun createRotationalTransform(angle: Float, out: Mat22) {
            val c = MathUtils.cos(angle)
            val s = MathUtils.sin(angle)
            out.ex.x = c
            out.ey.x = -s
            out.ex.y = s
            out.ey.y = c
        }

        /**
         * Creates a matrix that represents a scaling transformation.
         *
         * @param scale the scaling factor
         * @return a new matrix representing the scaling
         */
        @JvmStatic
        fun createScaleTransform(scale: Float): Mat22 {
            val mat = Mat22()
            mat.ex.x = scale
            mat.ey.y = scale
            return mat
        }

        /**
         * Creates a matrix that represents a scaling transformation and stores it in the output matrix.
         *
         * @param scale the scaling factor
         * @param out the matrix to store the result in
         */
        @JvmStatic
        fun createScaleTransform(scale: Float, out: Mat22) {
            out.ex.x = scale
            out.ey.y = scale
        }
    }
}
