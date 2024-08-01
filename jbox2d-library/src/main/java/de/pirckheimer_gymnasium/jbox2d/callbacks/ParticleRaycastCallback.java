package de.pirckheimer_gymnasium.jbox2d.callbacks;

import de.pirckheimer_gymnasium.jbox2d.common.Vec2;

public interface ParticleRaycastCallback {
  /**
   * Called for each particle found in the query. See
   * {@link RayCastCallback#reportFixture(de.pirckheimer_gymnasium.jbox2d.dynamics.Fixture, Vec2, Vec2, float)} for
   * argument info.
   *
   * @param index
   * @param point
   * @param normal
   * @param fraction
   *    */
  float reportParticle(int index, Vec2 point, Vec2 normal, float fraction);

}
