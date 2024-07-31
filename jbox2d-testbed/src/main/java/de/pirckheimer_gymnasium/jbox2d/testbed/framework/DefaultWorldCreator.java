package de.pirckheimer_gymnasium.jbox2d.testbed.framework;

import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.dynamics.World;

public class DefaultWorldCreator implements WorldCreator {
  @Override
  public World createWorld(Vec2 gravity) {
    return new World(gravity);
  }
}
