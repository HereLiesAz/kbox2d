package de.pirckheimer_gymnasium.jbox2d.particle;

import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.pooling.normal.MutableStack;

public class VoronoiDiagram
{
    public static class Generator
    {
        final Vec2 center = new Vec2();

        int tag;
    }

    public static class VoronoiDiagramTask
    {
        int x, y, i;

        Generator m_generator;

        public VoronoiDiagramTask()
        {
        }

        public VoronoiDiagramTask(int x, int y, int i, Generator g)
        {
            this.x = x;
            this.y = y;
            this.i = i;
            m_generator = g;
        }

        public VoronoiDiagramTask set(int x, int y, int i, Generator g)
        {
            this.x = x;
            this.y = y;
            this.i = i;
            m_generator = g;
            return this;
        }
    }

    public static interface VoronoiDiagramCallback
    {
        void callback(int aTag, int bTag, int cTag);
    }

    private Generator[] generatorBuffer;

    private int generatorCount;

    private int countX, countY;

    // The diagram is an array of "pointers".
    private Generator[] diagram;

    public VoronoiDiagram(int generatorCapacity)
    {
        generatorBuffer = new Generator[generatorCapacity];
        for (int i = 0; i < generatorCapacity; i++)
        {
            generatorBuffer[i] = new Generator();
        }
        generatorCount = 0;
        countX = 0;
        countY = 0;
        diagram = null;
    }

    public void getNodes(VoronoiDiagramCallback callback)
    {
        for (int y = 0; y < countY - 1; y++)
        {
            for (int x = 0; x < countX - 1; x++)
            {
                int i = x + y * countX;
                Generator a = diagram[i];
                Generator b = diagram[i + 1];
                Generator c = diagram[i + countX];
                Generator d = diagram[i + 1 + countX];
                if (b != c)
                {
                    if (a != b && a != c)
                    {
                        callback.callback(a.tag, b.tag, c.tag);
                    }
                    if (d != b && d != c)
                    {
                        callback.callback(b.tag, d.tag, c.tag);
                    }
                }
            }
        }
    }

    public void addGenerator(Vec2 center, int tag)
    {
        Generator g = generatorBuffer[generatorCount++];
        g.center.x = center.x;
        g.center.y = center.y;
        g.tag = tag;
    }

    private final Vec2 lower = new Vec2();

    private final Vec2 upper = new Vec2();

    private MutableStack<VoronoiDiagramTask> taskPool = new MutableStack<VoronoiDiagram.VoronoiDiagramTask>(
            50)
    {
        @Override
        protected VoronoiDiagramTask newInstance()
        {
            return new VoronoiDiagramTask();
        }

        @Override
        protected VoronoiDiagramTask[] newArray(int size)
        {
            return new VoronoiDiagramTask[size];
        }
    };

    private final StackQueue<VoronoiDiagramTask> queue = new StackQueue<VoronoiDiagramTask>();

    public void generate(float radius)
    {
        assert (diagram == null);
        float inverseRadius = 1 / radius;
        lower.x = Float.MAX_VALUE;
        lower.y = Float.MAX_VALUE;
        upper.x = -Float.MAX_VALUE;
        upper.y = -Float.MAX_VALUE;
        for (int k = 0; k < generatorCount; k++)
        {
            Generator g = generatorBuffer[k];
            Vec2.minToOut(lower, g.center, lower);
            Vec2.maxToOut(upper, g.center, upper);
        }
        countX = 1 + (int) (inverseRadius * (upper.x - lower.x));
        countY = 1 + (int) (inverseRadius * (upper.y - lower.y));
        diagram = new Generator[countX * countY];
        queue.reset(new VoronoiDiagramTask[4 * countX * countX]);
        for (int k = 0; k < generatorCount; k++)
        {
            Generator g = generatorBuffer[k];
            g.center.x = inverseRadius * (g.center.x - lower.x);
            g.center.y = inverseRadius * (g.center.y - lower.y);
            int x = MathUtils.max(0,
                    MathUtils.min((int) g.center.x, countX - 1));
            int y = MathUtils.max(0,
                    MathUtils.min((int) g.center.y, countY - 1));
            queue.push(taskPool.pop().set(x, y, x + y * countX, g));
        }
        while (!queue.empty())
        {
            VoronoiDiagramTask front = queue.pop();
            int x = front.x;
            int y = front.y;
            int i = front.i;
            Generator g = front.m_generator;
            if (diagram[i] == null)
            {
                diagram[i] = g;
                if (x > 0)
                {
                    queue.push(taskPool.pop().set(x - 1, y, i - 1, g));
                }
                if (y > 0)
                {
                    queue.push(taskPool.pop().set(x, y - 1, i - countX, g));
                }
                if (x < countX - 1)
                {
                    queue.push(taskPool.pop().set(x + 1, y, i + 1, g));
                }
                if (y < countY - 1)
                {
                    queue.push(taskPool.pop().set(x, y + 1, i + countX, g));
                }
            }
            taskPool.push(front);
        }
        int maxIteration = countX + countY;
        for (int iteration = 0; iteration < maxIteration; iteration++)
        {
            for (int y = 0; y < countY; y++)
            {
                for (int x = 0; x < countX - 1; x++)
                {
                    int i = x + y * countX;
                    Generator a = diagram[i];
                    Generator b = diagram[i + 1];
                    if (a != b)
                    {
                        queue.push(taskPool.pop().set(x, y, i, b));
                        queue.push(taskPool.pop().set(x + 1, y, i + 1, a));
                    }
                }
            }
            for (int y = 0; y < countY - 1; y++)
            {
                for (int x = 0; x < countX; x++)
                {
                    int i = x + y * countX;
                    Generator a = diagram[i];
                    Generator b = diagram[i + countX];
                    if (a != b)
                    {
                        queue.push(taskPool.pop().set(x, y, i, b));
                        queue.push(taskPool.pop().set(x, y + 1, i + countX, a));
                    }
                }
            }
            boolean updated = false;
            while (!queue.empty())
            {
                VoronoiDiagramTask front = queue.pop();
                int x = front.x;
                int y = front.y;
                int i = front.i;
                Generator k = front.m_generator;
                Generator a = diagram[i];
                Generator b = k;
                if (a != b)
                {
                    float ax = a.center.x - x;
                    float ay = a.center.y - y;
                    float bx = b.center.x - x;
                    float by = b.center.y - y;
                    float a2 = ax * ax + ay * ay;
                    float b2 = bx * bx + by * by;
                    if (a2 > b2)
                    {
                        diagram[i] = b;
                        if (x > 0)
                        {
                            queue.push(taskPool.pop().set(x - 1, y, i - 1, b));
                        }
                        if (y > 0)
                        {
                            queue.push(taskPool.pop().set(x, y - 1, i - countX,
                                    b));
                        }
                        if (x < countX - 1)
                        {
                            queue.push(taskPool.pop().set(x + 1, y, i + 1, b));
                        }
                        if (y < countY - 1)
                        {
                            queue.push(taskPool.pop().set(x, y + 1, i + countX,
                                    b));
                        }
                        updated = true;
                    }
                }
                taskPool.push(front);
            }
            if (!updated)
            {
                break;
            }
        }
    }
}
