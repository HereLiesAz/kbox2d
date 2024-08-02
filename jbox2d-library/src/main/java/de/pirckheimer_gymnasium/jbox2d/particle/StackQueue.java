package de.pirckheimer_gymnasium.jbox2d.particle;

public class StackQueue<T>
{
    private T[] buffer;

    private int front;

    private int back;

    private int end;

    public StackQueue()
    {
    }

    public void reset(T[] buffer)
    {
        this.buffer = buffer;
        front = 0;
        back = 0;
        end = buffer.length;
    }

    public void push(T task)
    {
        if (back >= end)
        {
            System.arraycopy(buffer, front, buffer, 0, back - front);
            back -= front;
            front = 0;
            if (back >= end)
            {
                return;
            }
        }
        buffer[back++] = task;
    }

    public T pop()
    {
        assert (front < back);
        return buffer[front++];
    }

    public boolean empty()
    {
        return front >= back;
    }

    public T front()
    {
        return buffer[front];
    }
}
