package com.hereliesaz.kbox2d.callbacks

import com.hereliesaz.kbox2d.collision.RayCastInput

/**
 * callback for a dynamic tree
 * @author Daniel Murphy
 */
interface TreeRayCastCallback {
    /**
     *
     * @param input
     * @param nodeId
     * @return the fraction to the node
     */
    fun raycastCallback(input: RayCastInput, nodeId: Int): Float
}
