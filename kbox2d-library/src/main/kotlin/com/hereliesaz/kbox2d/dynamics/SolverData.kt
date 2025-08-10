package com.hereliesaz.kbox2d.dynamics

import com.hereliesaz.kbox2d.dynamics.contacts.Position
import com.hereliesaz.kbox2d.dynamics.contacts.Velocity

class SolverData {
    lateinit var step: TimeStep
    lateinit var positions: Array<Position>
    lateinit var velocities: Array<Velocity>
}
