package de.pirckheimer_gymnasium.jbox2d.common

import java.io.Serializable

/**
 * This describes the motion of a body/shape for TOI computation. Shapes are
 * defined with respect to the body origin, which may not coincide with the
 * center of mass. However, to support dynamics we must interpolate the center
 * of mass position.
 *
 * @author Daniel Murphy
 */
class Sweep : Serializable {

    /**
     * Local center of mass position
     */
    @JvmField
    val localCenter: Vec2 = Vec2()

    /**
     * Center world positions
     */
    @JvmField
    val c0: Vec2 = Vec2()
    @JvmField
    val c: Vec2 = Vec2()

    /**
     * World angles
     */
    @JvmField
    var a0: Float = 0f
    @JvmField
    var a: Float = 0f

    /**
     * Fraction of the current time step in the range [0,1] c0 and a0 are the
     * positions at alpha0.
     */
    @JvmField
    var alpha0: Float = 0f

    override fun toString(): String {
        var s = "Sweep:\nlocalCenter: $localCenter\n"
        s += "c0: $c0, c: $c\n"
        s += "a0: $a0, a: $a\n"
        s += "alpha0: $alpha0"
        return s
    }

    fun normalize() {
        val d = MathUtils.TWOPI * MathUtils.floor(a0 / MathUtils.TWOPI)
        a0 -= d
        a -= d
    }

    fun set(other: Sweep): Sweep {
        localCenter.set(other.localCenter)
        c0.set(other.c0)
        c.set(other.c)
        a0 = other.a0
        a = other.a
        alpha0 = other.alpha0
        return this
    }

    /**
     * Get the interpolated transform at a specific time.
     *
     * @param xf The result is placed here - must not be null.
     * @param beta The normalized time in [0,1].
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L688-L697
     */
    fun getTransform(xf: Transform, beta: Float) {
        xf.p.x = (1.0f - beta) * c0.x + beta * c.x
        xf.p.y = (1.0f - beta) * c0.y + beta * c.y
        val angle = (1.0f - beta) * a0 + beta * a
        xf.q.set(angle)
        // Shift to origin
        val q = xf.q
        xf.p.x -= q.c * localCenter.x - q.s * localCenter.y
        xf.p.y -= q.s * localCenter.x + q.c * localCenter.y
    }

    /**
     * Advance the sweep forward, yielding a new initial state.
     *
     * @param alpha The new initial time.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_math.h#L699-L706
     */
    fun advance(alpha: Float) {
        assert(alpha0 < 1.0f)
        val beta = (alpha - alpha0) / (1.0f - alpha0)
        c0.x += beta * (c.x - c0.x)
        c0.y += beta * (c.y - c0.y)
        a0 += beta * (a - a0)
        alpha0 = alpha
    }

    companion object {
        private const val serialVersionUID = 1L
    }
}
