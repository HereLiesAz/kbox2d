package com.hereliesaz.kbox2d.dynamics.contacts

import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.pooling.IWorldPool

// updated to rev 100
interface ContactCreator {
    fun contactCreateFcn(argPool: IWorldPool, fixtureA: Fixture, fixtureB: Fixture): Contact
    fun contactDestroyFcn(argPool: IWorldPool, contact: Contact)
}
