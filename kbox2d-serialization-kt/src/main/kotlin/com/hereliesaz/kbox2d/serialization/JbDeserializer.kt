/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package com.hereliesaz.kbox2d.serialization

import com.hereliesaz.kbox2d.collision.shapes.Shape
import com.hereliesaz.kbox2d.dynamics.Body
import com.hereliesaz.kbox2d.dynamics.Fixture
import com.hereliesaz.kbox2d.dynamics.World
import com.hereliesaz.kbox2d.dynamics.joints.Joint
import java.io.IOException
import java.io.InputStream

interface JbDeserializer {
    /**
     * Sets the object listener, which allows the user to process each physics
     * object with a tag to do any sort of custom logic.
     */
    fun setObjectListener(argListener: ObjectListener)


    /**
     * Sets a listener for unsupported exceptions instead of stopping the whole
     * deserialization process by throwing and exception.
     */
    fun setUnsupportedListener(argListener: UnsupportedListener)

    /**
     * Deserializes a world
     *
     * @param input
     * @throws IOException
     * @throws UnsupportedObjectException if a read physics object is
     * unsupported by this library
     * @see .setUnsupportedListener
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeWorld(input: InputStream): World

    /**
     * Deserializes a body
     *
     * @throws UnsupportedObjectException if a read physics object is
     * unsupported by this library
     * @see .setUnsupportedListener
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeBody(world: World, input: InputStream): Body

    /**
     * Deserializes a fixture
     *
     * @throws UnsupportedObjectException if a read physics object is
     * unsupported by this library
     * @see .setUnsupportedListener
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeFixture(body: Body, input: InputStream): Fixture

    /**
     * Deserializes a shape
     *
     * @param input
     * @throws IOException
     * @throws UnsupportedObjectException if a read physics object is
     * unsupported by this library
     * @see .setUnsupportedListener
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeShape(input: InputStream): Shape

    /**
     * Deserializes a joint
     *
     * @throws UnsupportedObjectException if a read physics object is
     * unsupported by this library
     * @see .setUnsupportedListener
     */
    @Throws(IOException::class, UnsupportedObjectException::class)
    fun deserializeJoint(world: World, input: InputStream,
                         bodyMap: Map<Int, Body>, jointMap: Map<Int, Joint>): Joint

    /**
     * Called for each physics object with a tag defined.
     *
     * @author Daniel Murphy
     */
    interface ObjectListener {
        fun processWorld(world: World, tag: Long?)
        fun processBody(body: Body, tag: Long?)
        fun processFixture(fixture: Fixture, tag: Long?)
        fun processShape(shape: Shape, tag: Long?)
        fun processJoint(joint: Joint, tag: Long?)
    }
}
