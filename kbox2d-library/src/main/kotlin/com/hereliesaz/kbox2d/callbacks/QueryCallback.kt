package com.hereliesaz.kbox2d.callbacks

import com.hereliesaz.kbox2d.dynamics.Fixture

/**
 * Callback class for AABB queries.
 * See `World.queryAABB`.
 * @author Daniel Murphy
 */
interface QueryCallback {

    /**
     * Called for each fixture found in the query AABB.
     * @param fixture
     * @return false to terminate the query.
     */
    fun reportFixture(fixture: Fixture): Boolean
}
