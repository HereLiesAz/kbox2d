package com.hereliesaz.kbox2d.callbacks

/**
 * Callback class for AABB queries. See
 * `World.queryAABB`.
 *
 * @author dmurph
 */
interface ParticleQueryCallback {
    /**
     * Called for each particle found in the query AABB.
     *
     * @return false to terminate the query.
     */
    fun reportParticle(index: Int): Boolean
}
