/*
 * Copyright (c) 2013, Daniel Murphy
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *  * Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
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

import de.pirckheimer_gymnasium.jbox2d.callbacks.DebugDraw;
import de.pirckheimer_gymnasium.jbox2d.callbacks.TreeCallback;
import de.pirckheimer_gymnasium.jbox2d.callbacks.TreeRayCastCallback;
import de.pirckheimer_gymnasium.jbox2d.collision.AABB;
import de.pirckheimer_gymnasium.jbox2d.collision.RayCastInput;
import de.pirckheimer_gymnasium.jbox2d.common.BufferUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Color3f;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;

/**
 * @author Daniel Murphy
 */
public class DynamicTreeFlatNodes implements BroadPhaseStrategy
{
    public static final int MAX_STACK_SIZE = 64;

    public static final int NULL_NODE = -1;

    public static final int INITIAL_BUFFER_LENGTH = 16;

    public int root;

    public AABB[] m_aabb;

    public Object[] userData;

    protected int[] parent;

    protected int[] child1;

    protected int[] child2;

    protected int[] height;

    private int nodeCount;

    private int nodeCapacity;

    private int freeList;

    private final Vec2[] drawVecs = new Vec2[4];

    public DynamicTreeFlatNodes()
    {
        root = NULL_NODE;
        nodeCount = 0;
        nodeCapacity = 16;
        expandBuffers(0, nodeCapacity);
        for (int i = 0; i < drawVecs.length; i++)
        {
            drawVecs[i] = new Vec2();
        }
    }

    private void expandBuffers(int oldSize, int newSize)
    {
        m_aabb = BufferUtils.reallocateBuffer(AABB.class, m_aabb, oldSize,
                newSize);
        userData = BufferUtils.reallocateBuffer(Object.class, userData, oldSize,
                newSize);
        parent = BufferUtils.reallocateBuffer(parent, oldSize, newSize);
        child1 = BufferUtils.reallocateBuffer(child1, oldSize, newSize);
        child2 = BufferUtils.reallocateBuffer(child2, oldSize, newSize);
        height = BufferUtils.reallocateBuffer(height, oldSize, newSize);
        // Build a linked list for the free list.
        for (int i = oldSize; i < newSize; i++)
        {
            m_aabb[i] = new AABB();
            parent[i] = (i == newSize - 1) ? NULL_NODE : i + 1;
            height[i] = -1;
            child1[i] = -1;
            child2[i] = -1;
        }
        freeList = oldSize;
    }

    @Override
    public final int createProxy(final AABB aabb, Object userData)
    {
        final int node = allocateNode();
        // Fatten the aabb
        final AABB nodeAABB = m_aabb[node];
        nodeAABB.lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
        nodeAABB.lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
        nodeAABB.upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
        nodeAABB.upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
        this.userData[node] = userData;
        insertLeaf(node);
        return node;
    }

    @Override
    public final void destroyProxy(int proxyId)
    {
        assert (0 <= proxyId && proxyId < nodeCapacity);
        assert (child1[proxyId] == NULL_NODE);
        removeLeaf(proxyId);
        freeNode(proxyId);
    }

    @Override
    public final boolean moveProxy(int proxyId, final AABB aabb,
            Vec2 displacement)
    {
        assert (0 <= proxyId && proxyId < nodeCapacity);
        final int node = proxyId;
        assert (child1[node] == NULL_NODE);
        final AABB nodeAABB = m_aabb[node];
        // if (nodeAABB.contains(aabb)) {
        if (nodeAABB.lowerBound.x <= aabb.lowerBound.x
                && nodeAABB.lowerBound.y <= aabb.lowerBound.y
                && aabb.upperBound.x <= nodeAABB.upperBound.x
                && aabb.upperBound.y <= nodeAABB.upperBound.y)
        {
            return false;
        }
        removeLeaf(node);
        // Extend AABB
        final Vec2 lowerBound = nodeAABB.lowerBound;
        final Vec2 upperBound = nodeAABB.upperBound;
        lowerBound.x = aabb.lowerBound.x - Settings.aabbExtension;
        lowerBound.y = aabb.lowerBound.y - Settings.aabbExtension;
        upperBound.x = aabb.upperBound.x + Settings.aabbExtension;
        upperBound.y = aabb.upperBound.y + Settings.aabbExtension;
        // Predict AABB displacement.
        final float dx = displacement.x * Settings.aabbMultiplier;
        final float dy = displacement.y * Settings.aabbMultiplier;
        if (dx < 0.0f)
        {
            lowerBound.x += dx;
        }
        else
        {
            upperBound.x += dx;
        }
        if (dy < 0.0f)
        {
            lowerBound.y += dy;
        }
        else
        {
            upperBound.y += dy;
        }
        insertLeaf(proxyId);
        return true;
    }

    @Override
    public final Object getUserData(int proxyId)
    {
        assert (0 <= proxyId && proxyId < nodeCount);
        return userData[proxyId];
    }

    @Override
    public final AABB getFatAABB(int proxyId)
    {
        assert (0 <= proxyId && proxyId < nodeCount);
        return m_aabb[proxyId];
    }

    private int[] nodeStack = new int[20];

    private int nodeStackIndex;

    @Override
    public final void query(TreeCallback callback, AABB aabb)
    {
        nodeStackIndex = 0;
        nodeStack[nodeStackIndex++] = root;
        while (nodeStackIndex > 0)
        {
            int node = nodeStack[--nodeStackIndex];
            if (node == NULL_NODE)
            {
                continue;
            }
            if (AABB.testOverlap(m_aabb[node], aabb))
            {
                int child1 = this.child1[node];
                if (child1 == NULL_NODE)
                {
                    boolean proceed = callback.treeCallback(node);
                    if (!proceed)
                    {
                        return;
                    }
                }
                else
                {
                    if (nodeStack.length - nodeStackIndex - 2 <= 0)
                    {
                        nodeStack = BufferUtils.reallocateBuffer(nodeStack,
                                nodeStack.length, nodeStack.length * 2);
                    }
                    nodeStack[nodeStackIndex++] = child1;
                    nodeStack[nodeStackIndex++] = child2[node];
                }
            }
        }
    }

    private final Vec2 r = new Vec2();

    private final AABB aabb = new AABB();

    private final RayCastInput subInput = new RayCastInput();

    @Override
    public void raycast(TreeRayCastCallback callback, RayCastInput input)
    {
        final Vec2 p1 = input.p1;
        final Vec2 p2 = input.p2;
        float p1x = p1.x, p2x = p2.x, p1y = p1.y, p2y = p2.y;
        float vx, vy;
        float rx, ry;
        float absVx, absVy;
        float cx, cy;
        float hx, hy;
        float tempx, tempy;
        r.x = p2x - p1x;
        r.y = p2y - p1y;
        assert ((r.x * r.x + r.y * r.y) > 0f);
        r.normalize();
        rx = r.x;
        ry = r.y;
        // v is perpendicular to the segment.
        vx = -1f * ry;
        vy = rx;
        absVx = MathUtils.abs(vx);
        absVy = MathUtils.abs(vy);
        // Separating axis for segment (Gino, p80).
        // |dot(v, p1 - c)| > dot(|v|, h)
        float maxFraction = input.maxFraction;
        // Build a bounding box for the segment.
        final AABB segAABB = aabb;
        // Vec2 t = p1 + maxFraction * (p2 - p1);
        // before inline
        // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
        // Vec2.minToOut(p1, temp, segAABB.lowerBound);
        // Vec2.maxToOut(p1, temp, segAABB.upperBound);
        tempx = (p2x - p1x) * maxFraction + p1x;
        tempy = (p2y - p1y) * maxFraction + p1y;
        segAABB.lowerBound.x = Math.min(p1x, tempx);
        segAABB.lowerBound.y = Math.min(p1y, tempy);
        segAABB.upperBound.x = Math.max(p1x, tempx);
        segAABB.upperBound.y = Math.max(p1y, tempy);
        // end inline
        nodeStackIndex = 0;
        nodeStack[nodeStackIndex++] = root;
        while (nodeStackIndex > 0)
        {
            int node = nodeStack[--nodeStackIndex] = root;
            if (node == NULL_NODE)
            {
                continue;
            }
            final AABB nodeAABB = m_aabb[node];
            if (!AABB.testOverlap(nodeAABB, segAABB))
            {
                continue;
            }
            // Separating axis for segment (Gino, p80).
            // |dot(v, p1 - c)| > dot(|v|, h)
            // node.aabb.getCenterToOut(c);
            // node.aabb.getExtentsToOut(h);
            cx = (nodeAABB.lowerBound.x + nodeAABB.upperBound.x) * .5f;
            cy = (nodeAABB.lowerBound.y + nodeAABB.upperBound.y) * .5f;
            hx = (nodeAABB.upperBound.x - nodeAABB.lowerBound.x) * .5f;
            hy = (nodeAABB.upperBound.y - nodeAABB.lowerBound.y) * .5f;
            tempx = p1x - cx;
            tempy = p1y - cy;
            float separation = MathUtils.abs(vx * tempx + vy * tempy)
                    - (absVx * hx + absVy * hy);
            if (separation > 0.0f)
            {
                continue;
            }
            int child1 = this.child1[node];
            if (child1 == NULL_NODE)
            {
                subInput.p1.x = p1x;
                subInput.p1.y = p1y;
                subInput.p2.x = p2x;
                subInput.p2.y = p2y;
                subInput.maxFraction = maxFraction;
                float value = callback.raycastCallback(subInput, node);
                if (value == 0.0f)
                {
                    // The client has terminated the ray cast.
                    return;
                }
                if (value > 0.0f)
                {
                    // Update segment bounding box.
                    maxFraction = value;
                    // temp.set(p2).subLocal(p1).mulLocal(maxFraction).addLocal(p1);
                    // Vec2.minToOut(p1, temp, segAABB.lowerBound);
                    // Vec2.maxToOut(p1, temp, segAABB.upperBound);
                    tempx = (p2x - p1x) * maxFraction + p1x;
                    tempy = (p2y - p1y) * maxFraction + p1y;
                    segAABB.lowerBound.x = Math.min(p1x, tempx);
                    segAABB.lowerBound.y = Math.min(p1y, tempy);
                    segAABB.upperBound.x = Math.max(p1x, tempx);
                    segAABB.upperBound.y = Math.max(p1y, tempy);
                }
            }
            else
            {
                nodeStack[nodeStackIndex++] = child1;
                nodeStack[nodeStackIndex++] = child2[node];
            }
        }
    }

    @Override
    public final int computeHeight()
    {
        return computeHeight(root);
    }

    private int computeHeight(int node)
    {
        assert (0 <= node && node < nodeCapacity);
        if (child1[node] == NULL_NODE)
        {
            return 0;
        }
        int height1 = computeHeight(child1[node]);
        int height2 = computeHeight(child2[node]);
        return 1 + MathUtils.max(height1, height2);
    }

    /**
     * Validate this tree. For testing.
     */
    public void validate()
    {
        validateStructure(root);
        validateMetrics(root);
        int freeCount = 0;
        int freeNode = freeList;
        while (freeNode != NULL_NODE)
        {
            assert (0 <= freeNode && freeNode < nodeCapacity);
            freeNode = parent[freeNode];
            ++freeCount;
        }
        assert (getHeight() == computeHeight());
        assert (nodeCount + freeCount == nodeCapacity);
    }

    @Override
    public int getHeight()
    {
        if (root == NULL_NODE)
        {
            return 0;
        }
        return height[root];
    }

    @Override
    public int getMaxBalance()
    {
        int maxBalance = 0;
        for (int i = 0; i < nodeCapacity; ++i)
        {
            if (height[i] <= 1)
            {
                continue;
            }
            assert (child1[i] != NULL_NODE);
            int child1 = this.child1[i];
            int child2 = this.child2[i];
            int balance = MathUtils.abs(height[child2] - height[child1]);
            maxBalance = MathUtils.max(maxBalance, balance);
        }
        return maxBalance;
    }

    @Override
    public float getAreaRatio()
    {
        if (root == NULL_NODE)
        {
            return 0.0f;
        }
        final int root = this.root;
        float rootArea = m_aabb[root].getPerimeter();
        float totalArea = 0.0f;
        for (int i = 0; i < nodeCapacity; ++i)
        {
            if (height[i] < 0)
            {
                // Free node in pool
                continue;
            }
            totalArea += m_aabb[i].getPerimeter();
        }
        return totalArea / rootArea;
    }
    // /**
    // * Build an optimal tree. Very expensive. For testing.
    // */
    // public void rebuildBottomUp() {
    // int[] nodes = new int[m_nodeCount];
    // int count = 0;
    //
    // // Build array of leaves. Free the rest.
    // for (int i = 0; i < m_nodeCapacity; ++i) {
    // if (m_nodes[i].height < 0) {
    // // free node in pool
    // continue;
    // }
    //
    // DynamicTreeNode node = m_nodes[i];
    // if (node.isLeaf()) {
    // node.parent = null;
    // nodes[count] = i;
    // ++count;
    // } else {
    // freeNode(node);
    // }
    // }
    //
    // AABB b = new AABB();
    // while (count > 1) {
    // float minCost = Float.MAX_VALUE;
    // int iMin = -1, jMin = -1;
    // for (int i = 0; i < count; ++i) {
    // AABB aabbi = m_nodes[nodes[i]].aabb;
    //
    // for (int j = i + 1; j < count; ++j) {
    // AABB aabbj = m_nodes[nodes[j]].aabb;
    // b.combine(aabbi, aabbj);
    // float cost = b.getPerimeter();
    // if (cost < minCost) {
    // iMin = i;
    // jMin = j;
    // minCost = cost;
    // }
    // }
    // }
    //
    // int index1 = nodes[iMin];
    // int index2 = nodes[jMin];
    // DynamicTreeNode child1 = m_nodes[index1];
    // DynamicTreeNode child2 = m_nodes[index2];
    //
    // DynamicTreeNode parent = allocateNode();
    // parent.child1 = child1;
    // parent.child2 = child2;
    // parent.height = 1 + MathUtils.max(child1.height, child2.height);
    // parent.aabb.combine(child1.aabb, child2.aabb);
    // parent.parent = null;
    //
    // child1.parent = parent;
    // child2.parent = parent;
    //
    // nodes[jMin] = nodes[count - 1];
    // nodes[iMin] = parent.id;
    // --count;
    // }
    //
    // m_root = m_nodes[nodes[0]];
    //
    // validate();
    // }

    private final int allocateNode()
    {
        if (freeList == NULL_NODE)
        {
            assert (nodeCount == nodeCapacity);
            nodeCapacity *= 2;
            expandBuffers(nodeCount, nodeCapacity);
        }
        assert (freeList != NULL_NODE);
        int node = freeList;
        freeList = parent[node];
        parent[node] = NULL_NODE;
        child1[node] = NULL_NODE;
        height[node] = 0;
        ++nodeCount;
        return node;
    }

    /**
     * returns a node to the pool
     */
    private final void freeNode(int node)
    {
        assert (node != NULL_NODE);
        assert (0 < nodeCount);
        parent[node] = freeList != NULL_NODE ? freeList : NULL_NODE;
        height[node] = -1;
        freeList = node;
        nodeCount--;
    }

    private final AABB combinedAABB = new AABB();

    private final void insertLeaf(int leaf)
    {
        if (root == NULL_NODE)
        {
            root = leaf;
            parent[root] = NULL_NODE;
            return;
        }
        // find the best sibling
        AABB leafAABB = m_aabb[leaf];
        int index = root;
        while (child1[index] != NULL_NODE)
        {
            final int node = index;
            int child1 = this.child1[node];
            int child2 = this.child2[node];
            final AABB nodeAABB = m_aabb[node];
            float area = nodeAABB.getPerimeter();
            combinedAABB.combine(nodeAABB, leafAABB);
            float combinedArea = combinedAABB.getPerimeter();
            // Cost of creating a new parent for this node and the new leaf
            float cost = 2.0f * combinedArea;
            // Minimum cost of pushing the leaf further down the tree
            float inheritanceCost = 2.0f * (combinedArea - area);
            // Cost of descending into child1
            float cost1;
            AABB child1AABB = m_aabb[child1];
            if (this.child1[child1] == NULL_NODE)
            {
                combinedAABB.combine(leafAABB, child1AABB);
                cost1 = combinedAABB.getPerimeter() + inheritanceCost;
            }
            else
            {
                combinedAABB.combine(leafAABB, child1AABB);
                float oldArea = child1AABB.getPerimeter();
                float newArea = combinedAABB.getPerimeter();
                cost1 = (newArea - oldArea) + inheritanceCost;
            }
            // Cost of descending into child2
            float cost2;
            AABB child2AABB = m_aabb[child2];
            if (this.child1[child2] == NULL_NODE)
            {
                combinedAABB.combine(leafAABB, child2AABB);
                cost2 = combinedAABB.getPerimeter() + inheritanceCost;
            }
            else
            {
                combinedAABB.combine(leafAABB, child2AABB);
                float oldArea = child2AABB.getPerimeter();
                float newArea = combinedAABB.getPerimeter();
                cost2 = newArea - oldArea + inheritanceCost;
            }
            // Descend according to the minimum cost.
            if (cost < cost1 && cost < cost2)
            {
                break;
            }
            // Descend
            if (cost1 < cost2)
            {
                index = child1;
            }
            else
            {
                index = child2;
            }
        }
        int sibling = index;
        int oldParent = parent[sibling];
        final int newParent = allocateNode();
        parent[newParent] = oldParent;
        userData[newParent] = null;
        m_aabb[newParent].combine(leafAABB, m_aabb[sibling]);
        height[newParent] = height[sibling] + 1;
        if (oldParent != NULL_NODE)
        {
            // The sibling was not the root.
            if (child1[oldParent] == sibling)
            {
                child1[oldParent] = newParent;
            }
            else
            {
                child2[oldParent] = newParent;
            }
            child1[newParent] = sibling;
            child2[newParent] = leaf;
            parent[sibling] = newParent;
            parent[leaf] = newParent;
        }
        else
        {
            // The sibling was the root.
            child1[newParent] = sibling;
            child2[newParent] = leaf;
            parent[sibling] = newParent;
            parent[leaf] = newParent;
            root = newParent;
        }
        // Walk back up the tree fixing heights and AABBs
        index = parent[leaf];
        while (index != NULL_NODE)
        {
            index = balance(index);
            int child1 = this.child1[index];
            int child2 = this.child2[index];
            assert (child1 != NULL_NODE);
            assert (child2 != NULL_NODE);
            height[index] = 1 + MathUtils.max(height[child1], height[child2]);
            m_aabb[index].combine(m_aabb[child1], m_aabb[child2]);
            index = parent[index];
        }
        // validate();
    }

    private final void removeLeaf(int leaf)
    {
        if (leaf == root)
        {
            root = NULL_NODE;
            return;
        }
        int parent = this.parent[leaf];
        int grandParent = this.parent[parent];
        int parentChild1 = child1[parent];
        int parentChild2 = child2[parent];
        int sibling;
        if (parentChild1 == leaf)
        {
            sibling = parentChild2;
        }
        else
        {
            sibling = parentChild1;
        }
        if (grandParent != NULL_NODE)
        {
            // Destroy parent and connect sibling to grandParent.
            if (child1[grandParent] == parent)
            {
                child1[grandParent] = sibling;
            }
            else
            {
                child2[grandParent] = sibling;
            }
            this.parent[sibling] = grandParent;
            freeNode(parent);
            // Adjust ancestor bounds.
            int index = grandParent;
            while (index != NULL_NODE)
            {
                index = balance(index);
                int child1 = this.child1[index];
                int child2 = this.child2[index];
                m_aabb[index].combine(m_aabb[child1], m_aabb[child2]);
                height[index] = 1
                        + MathUtils.max(height[child1], height[child2]);
                index = this.parent[index];
            }
        }
        else
        {
            root = sibling;
            this.parent[sibling] = NULL_NODE;
            freeNode(parent);
        }
        // validate();
    }

    // Perform a left or right rotation if node A is imbalanced.
    // Returns the new root index.
    private int balance(int iA)
    {
        assert (iA != NULL_NODE);
        int A = iA;
        if (child1[A] == NULL_NODE || height[A] < 2)
        {
            return iA;
        }
        int iB = child1[A];
        int iC = child2[A];
        assert (0 <= iB && iB < nodeCapacity);
        assert (0 <= iC && iC < nodeCapacity);
        int B = iB;
        int C = iC;
        int balance = height[C] - height[B];
        // Rotate C up
        if (balance > 1)
        {
            int iF = child1[C];
            int iG = child2[C];
            int F = iF;
            int G = iG;
            // assert (F != null);
            // assert (G != null);
            assert (0 <= iF && iF < nodeCapacity);
            assert (0 <= iG && iG < nodeCapacity);
            // Swap A and C
            child1[C] = iA;
            int cParent = parent[C] = parent[A];
            parent[A] = iC;
            // A's old parent should point to C
            if (cParent != NULL_NODE)
            {
                if (child1[cParent] == iA)
                {
                    child1[cParent] = iC;
                }
                else
                {
                    assert (child2[cParent] == iA);
                    child2[cParent] = iC;
                }
            }
            else
            {
                root = iC;
            }
            // Rotate
            if (height[F] > height[G])
            {
                child2[C] = iF;
                child2[A] = iG;
                parent[G] = iA;
                m_aabb[A].combine(m_aabb[B], m_aabb[G]);
                m_aabb[C].combine(m_aabb[A], m_aabb[F]);
                height[A] = 1 + MathUtils.max(height[B], height[G]);
                height[C] = 1 + MathUtils.max(height[A], height[F]);
            }
            else
            {
                child2[C] = iG;
                child2[A] = iF;
                parent[F] = iA;
                m_aabb[A].combine(m_aabb[B], m_aabb[F]);
                m_aabb[C].combine(m_aabb[A], m_aabb[G]);
                height[A] = 1 + MathUtils.max(height[B], height[F]);
                height[C] = 1 + MathUtils.max(height[A], height[G]);
            }
            return iC;
        }
        // Rotate B up
        if (balance < -1)
        {
            int iD = child1[B];
            int iE = child2[B];
            int D = iD;
            int E = iE;
            assert (0 <= iD && iD < nodeCapacity);
            assert (0 <= iE && iE < nodeCapacity);
            // Swap A and B
            child1[B] = iA;
            int Bparent = parent[B] = parent[A];
            parent[A] = iB;
            // A's old parent should point to B
            if (Bparent != NULL_NODE)
            {
                if (child1[Bparent] == iA)
                {
                    child1[Bparent] = iB;
                }
                else
                {
                    assert (child2[Bparent] == iA);
                    child2[Bparent] = iB;
                }
            }
            else
            {
                root = iB;
            }
            // Rotate
            if (height[D] > height[E])
            {
                child2[B] = iD;
                child1[A] = iE;
                parent[E] = iA;
                m_aabb[A].combine(m_aabb[C], m_aabb[E]);
                m_aabb[B].combine(m_aabb[A], m_aabb[D]);
                height[A] = 1 + MathUtils.max(height[C], height[E]);
                height[B] = 1 + MathUtils.max(height[A], height[D]);
            }
            else
            {
                child2[B] = iE;
                child1[A] = iD;
                parent[D] = iA;
                m_aabb[A].combine(m_aabb[C], m_aabb[D]);
                m_aabb[B].combine(m_aabb[A], m_aabb[E]);
                height[A] = 1 + MathUtils.max(height[C], height[D]);
                height[B] = 1 + MathUtils.max(height[A], height[E]);
            }
            return iB;
        }
        return iA;
    }

    private void validateStructure(int node)
    {
        if (node == NULL_NODE)
        {
            return;
        }
        if (node == root)
        {
            assert (parent[node] == NULL_NODE);
        }
        int child1 = this.child1[node];
        int child2 = this.child2[node];
        if (child1 == NULL_NODE)
        {
            assert (child1 == NULL_NODE);
            assert (child2 == NULL_NODE);
            assert (height[node] == 0);
            return;
        }
        assert (child1 != NULL_NODE && 0 <= child1 && child1 < nodeCapacity);
        assert (child2 != NULL_NODE && 0 <= child2 && child2 < nodeCapacity);
        assert (parent[child1] == node);
        assert (parent[child2] == node);
        validateStructure(child1);
        validateStructure(child2);
    }

    private void validateMetrics(int node)
    {
        if (node == NULL_NODE)
        {
            return;
        }
        int child1 = this.child1[node];
        int child2 = this.child2[node];
        if (child1 == NULL_NODE)
        {
            assert (child1 == NULL_NODE);
            assert (child2 == NULL_NODE);
            assert (height[node] == 0);
            return;
        }
        assert (child1 != NULL_NODE && 0 <= child1 && child1 < nodeCapacity);
        assert (child2 != child1 && 0 <= child2 && child2 < nodeCapacity);
        int height1 = height[child1];
        int height2 = height[child2];
        int height;
        height = 1 + MathUtils.max(height1, height2);
        assert (this.height[node] == height);
        AABB aabb = new AABB();
        aabb.combine(m_aabb[child1], m_aabb[child2]);
        assert (aabb.lowerBound.equals(m_aabb[node].lowerBound));
        assert (aabb.upperBound.equals(m_aabb[node].upperBound));
        validateMetrics(child1);
        validateMetrics(child2);
    }

    @Override
    public void drawTree(DebugDraw argDraw)
    {
        if (root == NULL_NODE)
        {
            return;
        }
        int height = computeHeight();
        drawTree(argDraw, root, 0, height);
    }

    private final Color3f color = new Color3f();

    private final Vec2 textVec = new Vec2();

    public void drawTree(DebugDraw argDraw, int node, int spot, int height)
    {
        AABB a = m_aabb[node];
        a.getVertices(drawVecs);
        color.set(1, (height - spot) * 1f / height,
                (height - spot) * 1f / height);
        argDraw.drawPolygon(drawVecs, 4, color);
        argDraw.getViewportTranform().getWorldToScreen(a.upperBound, textVec);
        argDraw.drawString(textVec.x, textVec.y,
                node + "-" + (spot + 1) + "/" + height, color);
        int c1 = child1[node];
        int c2 = child2[node];
        if (c1 != NULL_NODE)
        {
            drawTree(argDraw, c1, spot + 1, height);
        }
        if (c2 != NULL_NODE)
        {
            drawTree(argDraw, c2, spot + 1, height);
        }
    }
}
