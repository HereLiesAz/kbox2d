package de.pirckheimer_gymnasium.jbox2d.profile.worlds;

import de.pirckheimer_gymnasium.jbox2d.dynamics.World;

public interface PerformanceTestWorld
{
    void setupWorld(World world);

    void step();
}
