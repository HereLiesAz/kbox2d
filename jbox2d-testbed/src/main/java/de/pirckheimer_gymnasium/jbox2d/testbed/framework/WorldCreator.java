package de.pirckheimer_gymnasium.jbox2d.testbed.framework;

import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.World;

public interface WorldCreator
{
    World createWorld(Vec2 gravity);
}
