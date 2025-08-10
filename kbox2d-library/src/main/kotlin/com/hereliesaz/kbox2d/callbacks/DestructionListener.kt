package com.hereliesaz.kbox2d.callbacks

import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.dynamics.joints.Joint

/**
 * Joints and fixtures are destroyed when their associated
 * body is destroyed. Implement this listener so that you
 * may nullify references to these joints and shapes.
 * @author Daniel Murphy
 */
interface DestructionListener {

    /**
     * Called when any joint is about to be destroyed due
     * to the destruction of one of its attached bodies.
     * @param joint
     */
    fun sayGoodbye(joint: Joint)

    /**
     * Called when any fixture is about to be destroyed due
     * to the destruction of its parent body.
     * @param fixture
     */
    fun sayGoodbye(fixture: Fixture)
}
