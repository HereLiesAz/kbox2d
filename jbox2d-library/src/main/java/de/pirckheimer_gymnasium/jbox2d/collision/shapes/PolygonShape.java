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
package de.pirckheimer_gymnasium.jbox2d.collision.shapes;

import de.pirckheimer_gymnasium.jbox2d.collision.AABB;
import de.pirckheimer_gymnasium.jbox2d.collision.RayCastInput;
import de.pirckheimer_gymnasium.jbox2d.collision.RayCastOutput;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Rot;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Transform;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.pooling.arrays.IntArray;
import de.pirckheimer_gymnasium.jbox2d.pooling.arrays.Vec2Array;

/**
 * A convex polygon shape. Polygons have a maximum number of vertices equal to
 * _maxPolygonVertices. In most cases you should not need many vertices for a
 * convex polygon.
 *
 * @permalink https://github.com/erincatto/box2d/blob/main/src/collision/b2_polygon_shape.cpp
 *
 * @author Daniel Murphy
 */
public class PolygonShape extends Shape
{
    /** Dump lots of debug information. */
    private final static boolean debug = false;

    /**
     * Local position of the shape centroid in parent body frame.
     */
    public final Vec2 centroid = new Vec2();

    /**
     * The vertices of the shape. Note: use getVertexCount(), not
     * vertices.length, to get number of active vertices.
     */
    public final Vec2 vertices[];

    /**
     * The normals of the shape. Note: use getVertexCount(), not normals.length,
     * to get number of active normals.
     */
    public final Vec2 normals[];

    /**
     * Number of active vertices in the shape.
     */
    public int count;

    // pooling
    private final Vec2 pool1 = new Vec2();

    private final Vec2 pool2 = new Vec2();

    private final Vec2 pool3 = new Vec2();

    private final Vec2 pool4 = new Vec2();

    private Transform poolt1 = new Transform();

    public PolygonShape()
    {
        super(ShapeType.POLYGON);
        count = 0;
        vertices = new Vec2[Settings.maxPolygonVertices];
        for (int i = 0; i < vertices.length; i++)
        {
            vertices[i] = new Vec2();
        }
        normals = new Vec2[Settings.maxPolygonVertices];
        for (int i = 0; i < normals.length; i++)
        {
            normals[i] = new Vec2();
        }
        setRadius(Settings.polygonRadius);
        centroid.setZero();
    }

    public final Shape clone()
    {
        PolygonShape shape = new PolygonShape();
        shape.centroid.set(this.centroid);
        for (int i = 0; i < shape.normals.length; i++)
        {
            shape.normals[i].set(normals[i]);
            shape.vertices[i].set(vertices[i]);
        }
        shape.setRadius(this.getRadius());
        shape.count = this.count;
        return shape;
    }

    /**
     * Create a convex hull from the given array of points. The count must be in
     * the range [3, Settings.maxPolygonVertices].
     *
     * @warning the points may be re-ordered, even if they form a convex
     *          polygon.
     * @warning collinear points are removed.
     */
    public final void set(final Vec2[] vertices, final int count)
    {
        set(vertices, count, null, null);
    }

    /**
     * Create a convex hull from the given array of points. The count must be in
     * the range [3, Settings.maxPolygonVertices]. This method takes an
     * arraypool for pooling.
     *
     * @warning the points may be re-ordered, even if they form a convex
     *          polygon.
     * @warning collinear points are removed.
     */
    public final void set(final Vec2[] verts, final int num,
            final Vec2Array vecPool, final IntArray intPool)
    {
        assert (3 <= num && num <= Settings.maxPolygonVertices);
        int n = MathUtils.min(num, Settings.maxPolygonVertices);
        // Perform welding and copy vertices into local buffer.
        Vec2[] ps = (vecPool != null) ? vecPool.get(Settings.maxPolygonVertices)
                : new Vec2[Settings.maxPolygonVertices];
        int tempCount = 0;
        for (int i = 0; i < n; ++i)
        {
            Vec2 v = verts[i];
            boolean unique = true;
            for (int j = 0; j < tempCount; ++j)
            {
                if (MathUtils.distanceSquared(v, ps[j]) < 0.5f
                        * Settings.linearSlop)
                {
                    unique = false;
                    break;
                }
            }
            if (unique)
            {
                ps[tempCount++] = v;
            }
        }
        n = tempCount;
        if (n < 3)
        {
            // Polygon is degenerate.
            assert (false);
            setAsBox(1.0f, 1.0f);
            return;
        }
        // Create the convex hull using the Gift wrapping algorithm
        // http://en.wikipedia.org/wiki/Gift_wrapping_algorithm
        // Find the right most point on the hull
        int i0 = 0;
        float x0 = ps[0].x;
        for (int i = 1; i < n; ++i)
        {
            float x = ps[i].x;
            if (x > x0 || (x == x0 && ps[i].y < ps[i0].y))
            {
                i0 = i;
                x0 = x;
            }
        }
        int[] hull = (intPool != null)
                ? intPool.get(Settings.maxPolygonVertices)
                : new int[Settings.maxPolygonVertices];
        int m = 0;
        int ih = i0;
        while (true)
        {
            hull[m] = ih;
            int ie = 0;
            for (int j = 1; j < n; ++j)
            {
                if (ie == ih)
                {
                    ie = j;
                    continue;
                }
                Vec2 r = pool1.set(ps[ie]).subLocal(ps[hull[m]]);
                Vec2 v = pool2.set(ps[j]).subLocal(ps[hull[m]]);
                float c = Vec2.cross(r, v);
                if (c < 0.0f)
                {
                    ie = j;
                }
                // Collinearity check
                if (c == 0.0f && v.lengthSquared() > r.lengthSquared())
                {
                    ie = j;
                }
            }
            ++m;
            ih = ie;
            if (ie == i0)
            {
                break;
            }
        }
        this.count = m;
        // Copy vertices.
        for (int i = 0; i < count; ++i)
        {
            if (vertices[i] == null)
            {
                vertices[i] = new Vec2();
            }
            vertices[i].set(ps[hull[i]]);
        }
        final Vec2 edge = pool1;
        // Compute normals. Ensure the edges have non-zero length.
        for (int i = 0; i < count; ++i)
        {
            final int i1 = i;
            final int i2 = i + 1 < count ? i + 1 : 0;
            edge.set(vertices[i2]).subLocal(vertices[i1]);
            assert (edge.lengthSquared() > Settings.EPSILON * Settings.EPSILON);
            Vec2.crossToOutUnsafe(edge, 1f, normals[i]);
            normals[i].normalize();
        }
        // Compute the polygon centroid.
        computeCentroidToOut(vertices, count, centroid);
    }

    /**
     * Build vertices to represent an axis-aligned box.
     *
     * @param hx The half-width.
     * @param hy The half-height.
     */
    public final void setAsBox(final float hx, final float hy)
    {
        count = 4;
        vertices[0].set(-hx, -hy);
        vertices[1].set(hx, -hy);
        vertices[2].set(hx, hy);
        vertices[3].set(-hx, hy);
        normals[0].set(0.0f, -1.0f);
        normals[1].set(1.0f, 0.0f);
        normals[2].set(0.0f, 1.0f);
        normals[3].set(-1.0f, 0.0f);
        centroid.setZero();
    }

    /**
     * Build vertices to represent an oriented box.
     *
     * @param hx     The half-width.
     * @param hy     The half-height.
     * @param center The center of the box in local coordinates.
     * @param angle  The rotation of the box in local coordinates.
     */
    public final void setAsBox(final float hx, final float hy,
            final Vec2 center, final float angle)
    {
        count = 4;
        vertices[0].set(-hx, -hy);
        vertices[1].set(hx, -hy);
        vertices[2].set(hx, hy);
        vertices[3].set(-hx, hy);
        normals[0].set(0.0f, -1.0f);
        normals[1].set(1.0f, 0.0f);
        normals[2].set(0.0f, 1.0f);
        normals[3].set(-1.0f, 0.0f);
        centroid.set(center);
        final Transform xf = poolt1;
        xf.p.set(center);
        xf.q.set(angle);
        // Transform vertices and normals.
        for (int i = 0; i < count; ++i)
        {
            Transform.mulToOut(xf, vertices[i], vertices[i]);
            Rot.mulToOut(xf.q, normals[i], normals[i]);
        }
    }

    public int getChildCount()
    {
        return 1;
    }

    @Override
    public final boolean testPoint(final Transform xf, final Vec2 p)
    {
        float tempx, tempy;
        final Rot xfq = xf.q;
        tempx = p.x - xf.p.x;
        tempy = p.y - xf.p.y;
        final float pLocalx = xfq.c * tempx + xfq.s * tempy;
        final float pLocaly = -xfq.s * tempx + xfq.c * tempy;
        if (debug)
        {
            System.out.println("--testPoint debug--");
            System.out.println("Vertices: ");
            for (int i = 0; i < count; ++i)
            {
                System.out.println(vertices[i]);
            }
            System.out.println("pLocal: " + pLocalx + ", " + pLocaly);
        }
        for (int i = 0; i < count; ++i)
        {
            Vec2 vertex = vertices[i];
            Vec2 normal = normals[i];
            tempx = pLocalx - vertex.x;
            tempy = pLocaly - vertex.y;
            final float dot = normal.x * tempx + normal.y * tempy;
            if (dot > 0.0f)
            {
                return false;
            }
        }
        return true;
    }

    @Override
    public final void computeAABB(final AABB aabb, final Transform xf,
            int childIndex)
    {
        final Vec2 lower = aabb.lowerBound;
        final Vec2 upper = aabb.upperBound;
        final Vec2 v1 = vertices[0];
        final float xfqc = xf.q.c;
        final float xfqs = xf.q.s;
        final float xfpx = xf.p.x;
        final float xfpy = xf.p.y;
        lower.x = (xfqc * v1.x - xfqs * v1.y) + xfpx;
        lower.y = (xfqs * v1.x + xfqc * v1.y) + xfpy;
        upper.x = lower.x;
        upper.y = lower.y;
        for (int i = 1; i < count; ++i)
        {
            Vec2 v2 = vertices[i];
            // Vec2 v = Mul(xf, vertices[i]);
            float vx = (xfqc * v2.x - xfqs * v2.y) + xfpx;
            float vy = (xfqs * v2.x + xfqc * v2.y) + xfpy;
            lower.x = Math.min(lower.x, vx);
            lower.y = Math.min(lower.y, vy);
            upper.x = Math.max(upper.x, vx);
            upper.y = Math.max(upper.y, vy);
        }
        lower.x -= radius;
        lower.y -= radius;
        upper.x += radius;
        upper.y += radius;
    }

    /**
     * Get the vertex count.
     *
     */
    public final int getVertexCount()
    {
        return count;
    }

    /**
     * Get a vertex by index.
     *
     * @param index
     */
    public final Vec2 getVertex(final int index)
    {
        assert (0 <= index && index < count);
        return vertices[index];
    }

    @Override
    public float computeDistanceToOut(Transform xf, Vec2 p, int childIndex,
            Vec2 normalOut)
    {
        float xfqc = xf.q.c;
        float xfqs = xf.q.s;
        float tx = p.x - xf.p.x;
        float ty = p.y - xf.p.y;
        float pLocalX = xfqc * tx + xfqs * ty;
        float pLocalY = -xfqs * tx + xfqc * ty;
        float maxDistance = -Float.MAX_VALUE;
        float normalForMaxDistanceX = pLocalX;
        float normalForMaxDistanceY = pLocalY;
        for (int i = 0; i < count; ++i)
        {
            Vec2 vertex = vertices[i];
            Vec2 normal = normals[i];
            tx = pLocalX - vertex.x;
            ty = pLocalY - vertex.y;
            float dot = normal.x * tx + normal.y * ty;
            if (dot > maxDistance)
            {
                maxDistance = dot;
                normalForMaxDistanceX = normal.x;
                normalForMaxDistanceY = normal.y;
            }
        }
        float distance;
        if (maxDistance > 0)
        {
            float minDistanceX = normalForMaxDistanceX;
            float minDistanceY = normalForMaxDistanceY;
            float minDistance2 = maxDistance * maxDistance;
            for (int i = 0; i < count; ++i)
            {
                Vec2 vertex = vertices[i];
                float distanceVecX = pLocalX - vertex.x;
                float distanceVecY = pLocalY - vertex.y;
                float distance2 = (distanceVecX * distanceVecX
                        + distanceVecY * distanceVecY);
                if (minDistance2 > distance2)
                {
                    minDistanceX = distanceVecX;
                    minDistanceY = distanceVecY;
                    minDistance2 = distance2;
                }
            }
            distance = MathUtils.sqrt(minDistance2);
            normalOut.x = xfqc * minDistanceX - xfqs * minDistanceY;
            normalOut.y = xfqs * minDistanceX + xfqc * minDistanceY;
            normalOut.normalize();
        }
        else
        {
            distance = maxDistance;
            normalOut.x = xfqc * normalForMaxDistanceX
                    - xfqs * normalForMaxDistanceY;
            normalOut.y = xfqs * normalForMaxDistanceX
                    + xfqc * normalForMaxDistanceY;
        }
        return distance;
    }

    @Override
    public final boolean raycast(RayCastOutput output, RayCastInput input,
            Transform xf, int childIndex)
    {
        final float xfqc = xf.q.c;
        final float xfqs = xf.q.s;
        final Vec2 xfp = xf.p;
        float tempx, tempy;
        // b2Vec2 p1 = b2MulT(xf.q, input.p1 - xf.p);
        // b2Vec2 p2 = b2MulT(xf.q, input.p2 - xf.p);
        tempx = input.p1.x - xfp.x;
        tempy = input.p1.y - xfp.y;
        final float p1x = xfqc * tempx + xfqs * tempy;
        final float p1y = -xfqs * tempx + xfqc * tempy;
        tempx = input.p2.x - xfp.x;
        tempy = input.p2.y - xfp.y;
        final float p2x = xfqc * tempx + xfqs * tempy;
        final float p2y = -xfqs * tempx + xfqc * tempy;
        final float dx = p2x - p1x;
        final float dy = p2y - p1y;
        float lower = 0, upper = input.maxFraction;
        int index = -1;
        for (int i = 0; i < count; ++i)
        {
            Vec2 normal = normals[i];
            Vec2 vertex = vertices[i];
            // p = p1 + a * d
            // dot(normal, p - v) = 0
            // dot(normal, p1 - v) + a * dot(normal, d) = 0
            float tempxn = vertex.x - p1x;
            float tempyn = vertex.y - p1y;
            final float numerator = normal.x * tempxn + normal.y * tempyn;
            final float denominator = normal.x * dx + normal.y * dy;
            if (denominator == 0.0f)
            {
                if (numerator < 0.0f)
                {
                    return false;
                }
            }
            else
            {
                // Note: we want this predicate without division:
                // lower < numerator / denominator, where denominator < 0
                // Since denominator < 0, we have to flip the inequality:
                // lower < numerator / denominator <==> denominator * lower >
                // numerator.
                if (denominator < 0.0f && numerator < lower * denominator)
                {
                    // Increase lower.
                    // The segment enters this half-space.
                    lower = numerator / denominator;
                    index = i;
                }
                else if (denominator > 0.0f && numerator < upper * denominator)
                {
                    // Decrease upper.
                    // The segment exits this half-space.
                    upper = numerator / denominator;
                }
            }
            if (upper < lower)
            {
                return false;
            }
        }
        assert (0.0f <= lower && lower <= input.maxFraction);
        if (index >= 0)
        {
            output.fraction = lower;
            // normal = Mul(xf.R, normals[index]);
            Vec2 normal = normals[index];
            Vec2 out = output.normal;
            out.x = xfqc * normal.x - xfqs * normal.y;
            out.y = xfqs * normal.x + xfqc * normal.y;
            return true;
        }
        return false;
    }

    public final void computeCentroidToOut(final Vec2[] vs, final int count,
            final Vec2 out)
    {
        assert (count >= 3);
        out.set(0.0f, 0.0f);
        float area = 0.0f;
        // pRef is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        final Vec2 pRef = pool1;
        pRef.setZero();
        final Vec2 e1 = pool2;
        final Vec2 e2 = pool3;
        final float inv3 = 1.0f / 3.0f;
        for (int i = 0; i < count; ++i)
        {
            // Triangle vertices.
            final Vec2 p1 = pRef;
            final Vec2 p2 = vs[i];
            final Vec2 p3 = i + 1 < count ? vs[i + 1] : vs[0];
            e1.set(p2).subLocal(p1);
            e2.set(p3).subLocal(p1);
            final float D = Vec2.cross(e1, e2);
            final float triangleArea = 0.5f * D;
            area += triangleArea;
            // Area weighted centroid
            e1.set(p1).addLocal(p2).addLocal(p3).mulLocal(triangleArea * inv3);
            out.addLocal(e1);
        }
        // Centroid
        assert (area > Settings.EPSILON);
        out.mulLocal(1.0f / area);
    }

    public void computeMass(final MassData massData, float density)
    {
        // Polygon mass, centroid, and inertia.
        // Let rho be the polygon density in mass per unit area.
        // Then:
        // mass = rho * int(dA)
        // centroid.x = (1/mass) * rho * int(x * dA)
        // centroid.y = (1/mass) * rho * int(y * dA)
        // I = rho * int((x*x + y*y) * dA)
        //
        // We can compute these integrals by summing all the integrals
        // for each triangle of the polygon. To evaluate the integral
        // for a single triangle, we make a change of variables to
        // the (u,v) coordinates of the triangle:
        // x = x0 + e1x * u + e2x * v
        // y = y0 + e1y * u + e2y * v
        // where 0 <= u && 0 <= v && u + v <= 1.
        //
        // We integrate u from [0,1-v] and then v from [0,1].
        // We also need to use the Jacobian of the transformation:
        // D = cross(e1, e2)
        //
        // Simplification: triangle centroid = (1/3) * (p1 + p2 + p3)
        //
        // The rest of the derivation is handled by computer algebra.
        assert (count >= 3);
        final Vec2 center = pool1;
        center.setZero();
        float area = 0.0f;
        float I = 0.0f;
        // pRef is the reference point for forming triangles.
        // It's location doesn't change the result (except for rounding error).
        final Vec2 s = pool2;
        s.setZero();
        // This code would put the reference point inside the polygon.
        for (int i = 0; i < count; ++i)
        {
            s.addLocal(vertices[i]);
        }
        s.mulLocal(1.0f / count);
        final float inv3 = 1.0f / 3.0f;
        final Vec2 e1 = pool3;
        final Vec2 e2 = pool4;
        for (int i = 0; i < count; ++i)
        {
            // Triangle vertices.
            e1.set(vertices[i]).subLocal(s);
            e2.set(s).negateLocal()
                    .addLocal(i + 1 < count ? vertices[i + 1] : vertices[0]);
            final float D = Vec2.cross(e1, e2);
            final float triangleArea = 0.5f * D;
            area += triangleArea;
            // Area weighted centroid
            center.x += triangleArea * inv3 * (e1.x + e2.x);
            center.y += triangleArea * inv3 * (e1.y + e2.y);
            final float ex1 = e1.x, ey1 = e1.y;
            final float ex2 = e2.x, ey2 = e2.y;
            float intX2 = ex1 * ex1 + ex2 * ex1 + ex2 * ex2;
            float intY2 = ey1 * ey1 + ey2 * ey1 + ey2 * ey2;
            I += (0.25f * inv3 * D) * (intX2 + intY2);
        }
        // Total mass
        massData.mass = density * area;
        // Center of mass
        assert (area > Settings.EPSILON);
        center.mulLocal(1.0f / area);
        massData.center.set(center).addLocal(s);
        // Inertia tensor relative to the local origin (point s)
        massData.I = I * density;
        // Shift to center of mass then to original body origin.
        massData.I += massData.mass
                * (Vec2.dot(massData.center, massData.center));
    }

    /**
     * Validate convexity. This is a very time consuming operation.
     */
    public boolean validate()
    {
        for (int i = 0; i < count; ++i)
        {
            int i1 = i;
            int i2 = i < count - 1 ? i1 + 1 : 0;
            Vec2 p = vertices[i1];
            Vec2 e = pool1.set(vertices[i2]).subLocal(p);
            for (int j = 0; j < count; ++j)
            {
                if (j == i1 || j == i2)
                {
                    continue;
                }
                Vec2 v = pool2.set(vertices[j]).subLocal(p);
                float c = Vec2.cross(e, v);
                if (c < 0.0f)
                {
                    return false;
                }
            }
        }
        return true;
    }

    /**
     * Get the vertices in local coordinates.
     */
    public Vec2[] getVertices()
    {
        return vertices;
    }

    /**
     * Get the edge normal vectors. There is one for each vertex.
     */
    public Vec2[] getNormals()
    {
        return normals;
    }

    /**
     * Get the centroid and apply the supplied transform.
     */
    public Vec2 centroid(final Transform xf)
    {
        return Transform.mul(xf, centroid);
    }

    /**
     * Get the centroid and apply the supplied transform.
     */
    public Vec2 centroidToOut(final Transform xf, final Vec2 out)
    {
        Transform.mulToOutUnsafe(xf, centroid, out);
        return out;
    }
}
