package de.pirckheimer_gymnasium.jbox2d.pooling

/**
 * Same functionality of a regular java.util stack. Object return order does not
 * matter.
 *
 * @author Daniel Murphy
 *
 * @param <E>
</E> */
interface DynamicStack<E> {
    /**
     * Pops an item off the stack
     */
    fun pop(): E

    /**
     * Pushes an item back on the stack
     */
    fun push(argObject: E)
}
