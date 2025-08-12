package de.pirckheimer_gymnasium.jbox2d.pooling.normal

/**
 * @author Daniel Murphy
 */
abstract class OrderedStack<E>(argStackSize: Int, argContainerSize: Int) {
    private val pool: Array<Any?>
    private var index: Int
    private val size: Int
    private val container: Array<Any?>

    init {
        size = argStackSize
        pool = arrayOfNulls(argStackSize)
        for (i in 0 until argStackSize) {
            pool[i] = newInstance()
        }
        index = 0
        container = arrayOfNulls(argContainerSize)
    }

    fun pop(): E {
        assert(index < size) { "End of stack reached, there is probably a leak somewhere" }
        return pool[index++] as E
    }

    fun pop(argNum: Int): Array<E> {
        assert(index + argNum < size) { "End of stack reached, there is probably a leak somewhere" }
        assert(argNum <= container.size) { "Container array is too small" }
        System.arraycopy(pool, index, container, 0, argNum)
        index += argNum
        return container as Array<E>
    }

    fun push(argNum: Int) {
        index -= argNum
        assert(index >= 0) { "Beginning of stack reached, push/pops are unmatched" }
    }

    /** Creates a new instance of the object contained by this stack.  */
    protected abstract fun newInstance(): E
}
