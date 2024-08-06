/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 * 	* Redistributions of source code must retain the above copyright notice,
 * 	  this list of conditions and the following disclaimer.
 * 	* Redistributions in binary form must reproduce the above copyright notice,
 * 	  this list of conditions and the following disclaimer in the documentation
 * 	  and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT
 * NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
package de.pirckheimer_gymnasium.jbox2d.collision.broadphase;

import java.util.Arrays;

import de.pirckheimer_gymnasium.jbox2d.callbacks.DebugDraw;
import de.pirckheimer_gymnasium.jbox2d.callbacks.PairCallback;
import de.pirckheimer_gymnasium.jbox2d.callbacks.TreeCallback;
import de.pirckheimer_gymnasium.jbox2d.callbacks.TreeRayCastCallback;
import de.pirckheimer_gymnasium.jbox2d.collision.AABB;
import de.pirckheimer_gymnasium.jbox2d.collision.RayCastInput;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;

/**
 * The broad-phase is used for computing pairs and performing volume queries and
 * ray casts. This broad-phase does not persist pairs. Instead, this reports
 * potentially new pairs. It is up to the client to consume the new pairs and to
 * track subsequent overlap.
 *
 * @author Daniel Murphy
 */
public class DefaultBroadPhaseBuffer implements TreeCallback, BroadPhase
{
    private final BroadPhaseStrategy tree;

    private int proxyCount;

    private int[] moveBuffer;

    private int moveCapacity;

    private int moveCount;

    private long[] pairBuffer;

    private int pairCapacity;

    private int pairCount;

    private int queryProxyId;

    public DefaultBroadPhaseBuffer(BroadPhaseStrategy strategy)
    {
        proxyCount = 0;
        pairCapacity = 16;
        pairCount = 0;
        pairBuffer = new long[pairCapacity];
        for (int i = 0; i < pairCapacity; i++)
        {
            pairBuffer[i] = 0;
        }
        moveCapacity = 16;
        moveCount = 0;
        moveBuffer = new int[moveCapacity];
        tree = strategy;
        queryProxyId = NULL_PROXY;
    }

    @Override
    public final int createProxy(final AABB aabb, Object userData)
    {
        int proxyId = tree.createProxy(aabb, userData);
        ++proxyCount;
        bufferMove(proxyId);
        return proxyId;
    }

    @Override
    public final void destroyProxy(int proxyId)
    {
        unbufferMove(proxyId);
        --proxyCount;
        tree.destroyProxy(proxyId);
    }

    @Override
    public final void moveProxy(int proxyId, final AABB aabb,
            final Vec2 displacement)
    {
        boolean buffer = tree.moveProxy(proxyId, aabb, displacement);
        if (buffer)
        {
            bufferMove(proxyId);
        }
    }

    @Override
    public void touchProxy(int proxyId)
    {
        bufferMove(proxyId);
    }

    @Override
    public Object getUserData(int proxyId)
    {
        return tree.getUserData(proxyId);
    }

    @Override
    public AABB getFatAABB(int proxyId)
    {
        return tree.getFatAABB(proxyId);
    }

    @Override
    public boolean testOverlap(int proxyIdA, int proxyIdB)
    {
        // return AABB.testOverlap(proxyA.aabb, proxyB.aabb);
        // return tree.overlap(proxyIdA, proxyIdB);
        final AABB a = tree.getFatAABB(proxyIdA);
        final AABB b = tree.getFatAABB(proxyIdB);
        if (b.lowerBound.x - a.upperBound.x > 0.0f
                || b.lowerBound.y - a.upperBound.y > 0.0f)
        {
            return false;
        }
        return !(a.lowerBound.x - b.upperBound.x > 0.0f)
                && !(a.lowerBound.y - b.upperBound.y > 0.0f);
    }

    @Override
    public final int getProxyCount()
    {
        return proxyCount;
    }

    @Override
    public void drawTree(DebugDraw argDraw)
    {
        tree.drawTree(argDraw);
    }

    @Override
    public final void updatePairs(PairCallback callback)
    {
        // Reset pair buffer
        pairCount = 0;
        // Perform tree queries for all moving proxies.
        for (int i = 0; i < moveCount; ++i)
        {
            queryProxyId = moveBuffer[i];
            if (queryProxyId == NULL_PROXY)
            {
                continue;
            }
            // We have to query the tree with the fat AABB so that
            // we don't fail to create a pair that may touch later.
            final AABB fatAABB = tree.getFatAABB(queryProxyId);
            // Query tree, create pairs and add them pair buffer.
            // log.debug("quering aabb: "+queryProxy.aabb);
            tree.query(this, fatAABB);
        }
        // log.debug("Number of pairs found: "+pairCount);
        // Reset move buffer
        moveCount = 0;
        // Sort the pair buffer to expose duplicates.
        Arrays.sort(pairBuffer, 0, pairCount);
        // Send the pairs back to the client.
        int i = 0;
        while (i < pairCount)
        {
            long primaryPair = pairBuffer[i];
            Object userDataA = tree.getUserData((int) (primaryPair >> 32));
            Object userDataB = tree.getUserData((int) (primaryPair));
            // log.debug("returning pair: "+userDataA+", "+userDataB);
            callback.addPair(userDataA, userDataB);
            ++i;
            // Skip any duplicate pairs.
            while (i < pairCount)
            {
                long pair = pairBuffer[i];
                if (pair != primaryPair)
                {
                    break;
                }
                ++i;
            }
        }
    }

    @Override
    public final void query(final TreeCallback callback, final AABB aabb)
    {
        tree.query(callback, aabb);
    }

    @Override
    public final void raycast(final TreeRayCastCallback callback,
            final RayCastInput input)
    {
        tree.raycast(callback, input);
    }

    @Override
    public final int getTreeHeight()
    {
        return tree.getHeight();
    }

    @Override
    public int getTreeBalance()
    {
        return tree.getMaxBalance();
    }

    @Override
    public float getTreeQuality()
    {
        return tree.getAreaRatio();
    }

    protected final void bufferMove(int proxyId)
    {
        if (moveCount == moveCapacity)
        {
            int[] old = moveBuffer;
            moveCapacity *= 2;
            moveBuffer = new int[moveCapacity];
            System.arraycopy(old, 0, moveBuffer, 0, old.length);
        }
        moveBuffer[moveCount] = proxyId;
        ++moveCount;
    }

    protected final void unbufferMove(int proxyId)
    {
        for (int i = 0; i < moveCount; i++)
        {
            if (moveBuffer[i] == proxyId)
            {
                moveBuffer[i] = NULL_PROXY;
            }
        }
    }

    /**
     * This is called from DynamicTree::query when we are gathering pairs.
     */
    public final boolean treeCallback(int proxyId)
    {
        // A proxy cannot form a pair with itself.
        if (proxyId == queryProxyId)
        {
            return true;
        }
        // Grow the pair buffer as needed.
        if (pairCount == pairCapacity)
        {
            long[] oldBuffer = pairBuffer;
            pairCapacity *= 2;
            pairBuffer = new long[pairCapacity];
            System.arraycopy(oldBuffer, 0, pairBuffer, 0, oldBuffer.length);
            for (int i = oldBuffer.length; i < pairCapacity; i++)
            {
                pairBuffer[i] = 0;
            }
        }
        if (proxyId < queryProxyId)
        {
            pairBuffer[pairCount] = ((long) proxyId << 32) | queryProxyId;
        }
        else
        {
            pairBuffer[pairCount] = ((long) queryProxyId << 32) | proxyId;
        }
        ++pairCount;
        return true;
    }
}
