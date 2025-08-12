package de.pirckheimer_gymnasium.jbox2d.pooling.normal

import de.pirckheimer_gymnasium.jbox2d.pooling.DynamicStack

abstract class MutableStack<E>(argInitSize: Int) : DynamicStack<E> {
    private var stack: Array<E?>? = null
    private var index: Int = 0
    private var size: Int = 0

    init {
        index = 0
        extendStack(argInitSize)
    }

    private fun extendStack(argSize: Int) {
        val newStack = newArray(argSize)
        if (stack != null) {
            System.arraycopy(stack, 0, newStack, 0, size)
        }
        for (i in newStack.indices) {
            newStack[i] = newInstance()
        }
        stack = newStack
        size = newStack.size
    }

    override fun pop(): E {
        if (index >= size) {
            extendStack(size * 2)
        }
        return stack!![index++]!!
    }

    override fun push(argObject: E) {
        assert(index > 0)
        stack!![--index] = argObject
    }

    /** Creates a new instance of the object contained by this stack.  */
    protected abstract fun newInstance(): E

    protected abstract fun newArray(size: Int): Array<E?>
}
