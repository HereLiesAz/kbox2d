package com.hereliesaz.kbox2d.common

/**
 * Timer for profiling
 *
 * @author Daniel
 */
class Timer {
    private var resetNanos: Long = 0

    init {
        reset()
    }

    fun reset() {
        resetNanos = System.nanoTime()
    }

    val milliseconds: Float
        get() = (System.nanoTime() - resetNanos) / 1000 * 1f / 1000
}
