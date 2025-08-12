package de.pirckheimer_gymnasium.jbox2d.pooling

/**
 * This stack assumes that when you push 'n' items back, you're pushing back the
 * last 'n' items popped.
 *
 * @author Daniel Murphy
 */
interface OrderedStack<E> {
    /**
     * Returns the next object in the pool
     */
    fun pop(): E

    /**
     * Returns the next 'argNum' objects in the pool in an array
     *
     * @return an array containing the next pool objects in items 0-argNum.
     * Array length and uniqueness not guaranteed.
     */
    fun pop(argNum: Int): Array<E>

    /**
     * Tells the stack to take back the last 'argNum' items
     */
    fun push(argNum: Int)
}
