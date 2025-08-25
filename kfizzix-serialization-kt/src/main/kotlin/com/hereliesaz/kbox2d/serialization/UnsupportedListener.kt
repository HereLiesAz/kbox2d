package com.hereliesaz.kbox2d.serialization

/**
 * Used to hear when an object is unsupported by the serializer or the
 * deserializer. This is so the de/serializer can keep going without throwing an
 * exception and stopping the entire thing
 *
 * @author Daniel Murphy
 */
interface UnsupportedListener {
    /**
     * Called when an object is unsupported by the de/serializer.
     *
     * @param argException The exception describing the error.
     * @return if the process should stop and the exception be thrown.
     */
    fun isUnsupported(argException: UnsupportedObjectException): Boolean
}
