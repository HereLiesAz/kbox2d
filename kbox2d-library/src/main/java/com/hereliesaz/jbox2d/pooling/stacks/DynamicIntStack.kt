package de.pirckheimer_gymnasium.jbox2d.pooling.stacks

class DynamicIntStack(initialSize: Int) {
    private var stack: IntArray
    private var size: Int
    var count: Int = 0
        private set

    init {
        stack = IntArray(initialSize)
        count = 0
        size = initialSize
    }

    fun reset() {
        count = 0
    }

    fun pop(): Int {
        assert(count > 0)
        return stack[--count]
    }

    fun push(i: Int) {
        if (count == size) {
            val old = stack
            stack = IntArray(size * 2)
            size = stack.size
            System.arraycopy(old, 0, stack, 0, old.size)
        }
        stack[count++] = i
    }
}
