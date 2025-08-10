package com.hereliesaz.kbox2d.callbacks

import com.hereliesaz.kbox2d.collision.broadphase.DynamicTree

// update to rev 100
/**
 * callback for [DynamicTree]
 * @author Daniel Murphy
 */
interface TreeCallback {

    /**
     * Callback from a query request.
     * @param proxyId the id of the proxy
     * @return if the query should be continued
     */
    fun treeCallback(proxyId: Int): Boolean
}
