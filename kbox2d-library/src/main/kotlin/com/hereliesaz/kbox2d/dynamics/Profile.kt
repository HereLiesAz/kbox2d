package com.hereliesaz.kbox2d.dynamics

import com.hereliesaz.kbox2d.common.MathUtils

class Profile {
    class ProfileEntry {
        var longAvg = 0f
        var shortAvg = 0f
        var min: Float
        var max: Float
        var accum = 0f

        init {
            min = Float.MAX_VALUE
            max = -Float.MAX_VALUE
        }

        fun record(value: Float) {
            longAvg = longAvg * (1 - LONG_FRACTION) + value * LONG_FRACTION
            shortAvg = shortAvg * (1 - SHORT_FRACTION) + value * SHORT_FRACTION
            min = MathUtils.min(value, min)
            max = MathUtils.max(value, max)
        }

        fun startAccum() {
            accum = 0f
        }

        fun accum(value: Float) {
            accum += value
        }

        fun endAccum() {
            record(accum)
        }

        override fun toString(): String {
            return String.format("%.2f (%.2f) [%.2f,%.2f]", shortAvg, longAvg, min, max)
        }
    }

    val step = ProfileEntry()
    val stepInit = ProfileEntry()
    val collide = ProfileEntry()
    val solveParticleSystem = ProfileEntry()
    val solve = ProfileEntry()
    val solveInit = ProfileEntry()
    val solveVelocity = ProfileEntry()
    val solvePosition = ProfileEntry()
    val broadphase = ProfileEntry()
    val solveTOI = ProfileEntry()
    fun toDebugStrings(strings: MutableList<String>) {
        strings.add("Profile:")
        strings.add(" step: $step")
        strings.add("  init: $stepInit")
        strings.add("  collide: $collide")
        strings.add("  particles: $solveParticleSystem")
        strings.add("  solve: $solve")
        strings.add("   solveInit: $solveInit")
        strings.add("   solveVelocity: $solveVelocity")
        strings.add("   solvePosition: $solvePosition")
        strings.add("   broadphase: $broadphase")
        strings.add("  solveTOI: $solveTOI")
    }

    companion object {
        private const val LONG_AVG_NUMS = 20
        private const val LONG_FRACTION = 1f / LONG_AVG_NUMS
        private const val SHORT_AVG_NUMS = 5
        private const val SHORT_FRACTION = 1f / SHORT_AVG_NUMS
    }
}
