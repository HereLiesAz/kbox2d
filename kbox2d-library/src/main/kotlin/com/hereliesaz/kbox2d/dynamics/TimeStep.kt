package com.hereliesaz.kbox2d.dynamics

//updated to rev 100
/**
 * This is an internal structure.
 */
class TimeStep {
    /** time step  */
    var dt = 0f

    /** inverse time step (0 if dt == 0).  */
    var inv_dt = 0f

    /** dt * inv_dt0  */
    var dtRatio = 0f
    var velocityIterations = 0
    var positionIterations = 0
    var warmStarting = false
}
