package de.pirckheimer_gymnasium.jbox2d.callbacks;

import de.pirckheimer_gymnasium.jbox2d.dynamics.World;

/**
 * Callback class for AABB queries. See
 * {@link World#queryAABB(QueryCallback, de.pirckheimer_gymnasium.jbox2d.collision.AABB)}.
 *
 * @author dmurph
 *
 */
public interface ParticleQueryCallback
{
    /**
     * Called for each particle found in the query AABB.
     *
     * @return false to terminate the query.
     */
    boolean reportParticle(int index);
}
