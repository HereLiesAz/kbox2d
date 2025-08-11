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

/**
 * Serializer for jbox2d, used to serialize any aspect of the physics world
 *
 * @author Daniel Murphy
 */
interface JbSerializer {
    /**
     * Sets the object signer for the serializer. This allows the user to
     * specify a 'tag' for each main physics object, which is then referenced
     * later at deserialization for the user.
     */
    fun setObjectSigner(signer: ObjectSigner)

    /**
     * Sets a listener for unsupported exception instead of stopping the whole
     * serialization process by throwing and exception.
     */
    fun setUnsupportedListener(listener: UnsupportedListener)

    /**
     * Serializes the world
     *
     * @param world
     * @throws UnsupportedObjectException if a physics object is
     * unsupported by this library.
     * @see .setUnsupportedListener
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(world: World): SerializationResult

    /**
     * Serializes a body
     *
     * @param body
     * @throws UnsupportedObjectException if a physics object is
     * unsupported by this library.
     * @see .setUnsupportedListener
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(body: Body): SerializationResult

    /**
     * Serializes a fixture
     *
     * @param fixture
     * @throws UnsupportedObjectException if a physics object
     * is unsupported by this library.
     * @see .setUnsupportedListener
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(fixture: Fixture): SerializationResult

    /**
     * Serializes a shape
     *
     * @param shape
     * @throws UnsupportedObjectException if a physics object is
     * unsupported by this library.
     * @see .setUnsupportedListener
     */
    @Throws(UnsupportedObjectException::class)
    fun serialize(shape: Shape): SerializationResult

    /**
     * Serializes joints. Joints need to reference bodies and sometimes other
     * joints.
     */
    fun serialize(joint: Joint, bodyIndexMap: Map<Body, Int>,
                  jointIndexMap: Map<Joint, Int>): SerializationResult

    /**
     * Interface that allows the serializer to look up tags for each object,
     * which can be used later during deserializing by the developer.
     *
     * @author Daniel Murphy
     */
    interface ObjectSigner {
        /**
         * @return The tag for the world. Can be null.
         */
        fun getTag(world: World): Long?

        /**
         * @return The tag for the body. Can be null.
         */
        fun getTag(body: Body): Long?

        /**
         * @return The tag for the shape. Can be null.
         */
        fun getTag(shape: Shape): Long?

        /**
         * @return The tag for the fixture. Can be null.
         */
        fun getTag(fixture: Fixture): Long?

        /**
         * @return The tag for the joint. Can be null.
         */
        fun getTag(joint: Joint): Long?
    }
}
