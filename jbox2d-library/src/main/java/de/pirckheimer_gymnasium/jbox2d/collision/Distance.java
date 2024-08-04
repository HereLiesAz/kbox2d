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
package de.pirckheimer_gymnasium.jbox2d.collision;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.ChainShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.CircleShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.EdgeShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.Shape;
import de.pirckheimer_gymnasium.jbox2d.common.MathUtils;
import de.pirckheimer_gymnasium.jbox2d.common.Rot;
import de.pirckheimer_gymnasium.jbox2d.common.Settings;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.common.Transform;

/**
 * This is non-static for faster pooling. To get an instance, use the
 * {@code SingletonPool}, don't construct a distance object.
 *
 * @author Daniel Murphy
 */
public class Distance
{
    public static final int MAX_ITERS = 20;

    public static int GJK_CALLS = 0;

    public static int GJK_ITERS = 0;

    public static int GJK_MAX_ITERS = 20;

    /**
     * GJK using Voronoi regions (Christer Ericson) and Barycentric coordinates.
     */
    private static class SimplexVertex
    {
        /**
         * support point in shapeA
         */
        public final Vec2 wA = new Vec2();

        /**
         * support point in shapeB
         */
        public final Vec2 wB = new Vec2();

        /**
         * wB - wA
         */
        public final Vec2 w = new Vec2();

        /**
         * barycentric coordinate for closest point
         */
        public float a;

        /**
         * wA index
         */
        public int indexA;

        /**
         * wB index
         */
        public int indexB;

        public void set(SimplexVertex sv)
        {
            wA.set(sv.wA);
            wB.set(sv.wB);
            w.set(sv.w);
            a = sv.a;
            indexA = sv.indexA;
            indexB = sv.indexB;
        }
    }

    /**
     * Used to warm start Distance. Set count to zero on first call.
     *
     * @author Daniel Murphy
     */
    public static class SimplexCache
    {
        /**
         * length or area
         */
        public float metric;

        public int count;

        /**
         * vertices on shape A
         */
        public final int[] indexA = new int[3];

        /**
         * vertices on shape B
         */
        public final int[] indexB = new int[3];

        public SimplexCache()
        {
            metric = 0;
            count = 0;
            indexA[0] = Integer.MAX_VALUE;
            indexA[1] = Integer.MAX_VALUE;
            indexA[2] = Integer.MAX_VALUE;
            indexB[0] = Integer.MAX_VALUE;
            indexB[1] = Integer.MAX_VALUE;
            indexB[2] = Integer.MAX_VALUE;
        }

        public void set(SimplexCache sc)
        {
            System.arraycopy(sc.indexA, 0, indexA, 0, indexA.length);
            System.arraycopy(sc.indexB, 0, indexB, 0, indexB.length);
            metric = sc.metric;
            count = sc.count;
        }
    }

    private static class Simplex
    {
        public final SimplexVertex v1 = new SimplexVertex();

        public final SimplexVertex v2 = new SimplexVertex();

        public final SimplexVertex v3 = new SimplexVertex();

        public final SimplexVertex[] vertices = { v1, v2, v3 };

        public int count;

        public void readCache(SimplexCache cache, DistanceProxy proxyA,
                Transform transformA, DistanceProxy proxyB,
                Transform transformB)
        {
            assert (cache.count <= 3);
            // Copy data from cache.
            count = cache.count;
            for (int i = 0; i < count; ++i)
            {
                SimplexVertex v = vertices[i];
                v.indexA = cache.indexA[i];
                v.indexB = cache.indexB[i];
                Vec2 wALocal = proxyA.getVertex(v.indexA);
                Vec2 wBLocal = proxyB.getVertex(v.indexB);
                Transform.mulToOutUnsafe(transformA, wALocal, v.wA);
                Transform.mulToOutUnsafe(transformB, wBLocal, v.wB);
                v.w.set(v.wB).subLocal(v.wA);
                v.a = 0.0f;
            }
            // Compute the new simplex metric, if it is substantially different
            // than
            // old metric then flush the simplex.
            if (count > 1)
            {
                float metric1 = cache.metric;
                float metric2 = getMetric();
                if (metric2 < 0.5f * metric1 || 2.0f * metric1 < metric2
                        || metric2 < Settings.EPSILON)
                {
                    // Reset the simplex.
                    count = 0;
                }
            }
            // If the cache is empty or invalid ...
            if (count == 0)
            {
                SimplexVertex v = vertices[0];
                v.indexA = 0;
                v.indexB = 0;
                Vec2 wALocal = proxyA.getVertex(0);
                Vec2 wBLocal = proxyB.getVertex(0);
                Transform.mulToOutUnsafe(transformA, wALocal, v.wA);
                Transform.mulToOutUnsafe(transformB, wBLocal, v.wB);
                v.w.set(v.wB).subLocal(v.wA);
                count = 1;
            }
        }

        public void writeCache(SimplexCache cache)
        {
            cache.metric = getMetric();
            cache.count = count;
            for (int i = 0; i < count; ++i)
            {
                cache.indexA[i] = (vertices[i].indexA);
                cache.indexB[i] = (vertices[i].indexB);
            }
        }

        private final Vec2 e12 = new Vec2();

        public final void getSearchDirection(final Vec2 out)
        {
            switch (count)
            {
            case 1:
                out.set(v1.w).negateLocal();
                return;

            case 2:
                e12.set(v2.w).subLocal(v1.w);
                // use out for a temp variable real quick
                out.set(v1.w).negateLocal();
                float sgn = Vec2.cross(e12, out);
                if (sgn > 0f)
                {
                    // Origin is left of e12.
                    Vec2.crossToOutUnsafe(1f, e12, out);
                }
                else
                {
                    // Origin is right of e12.
                    Vec2.crossToOutUnsafe(e12, 1f, out);
                }
                return;

            default:
                assert (false);
                out.setZero();
            }
        }

        // djm pooled
        private final Vec2 case2 = new Vec2();

        private final Vec2 case22 = new Vec2();

        /**
         * this returns pooled objects. don't keep or modify them
         *
         */
        public void getClosestPoint(final Vec2 out)
        {
            switch (count)
            {
            case 1:
                out.set(v1.w);
                return;

            case 2:
                case22.set(v2.w).mulLocal(v2.a);
                case2.set(v1.w).mulLocal(v1.a).addLocal(case22);
                out.set(case2);
                return;

            case 3:
                out.setZero();
                return;

            default:
                assert (false);
                out.setZero();
            }
        }

        // djm pooled, and from above
        private final Vec2 case3 = new Vec2();

        private final Vec2 case33 = new Vec2();

        public void getWitnessPoints(Vec2 pA, Vec2 pB)
        {
            switch (count)
            {
            case 1:
                pA.set(v1.wA);
                pB.set(v1.wB);
                break;

            case 2:
                case2.set(v1.wA).mulLocal(v1.a);
                pA.set(v2.wA).mulLocal(v2.a).addLocal(case2);
                // v1.a * v1.wA + v2.a * v2.wA;
                // *pB = v1.a * v1.wB + v2.a * v2.wB;
                case2.set(v1.wB).mulLocal(v1.a);
                pB.set(v2.wB).mulLocal(v2.a).addLocal(case2);
                break;

            case 3:
                pA.set(v1.wA).mulLocal(v1.a);
                case3.set(v2.wA).mulLocal(v2.a);
                case33.set(v3.wA).mulLocal(v3.a);
                pA.addLocal(case3).addLocal(case33);
                pB.set(pA);
                // *pA = v1.a * v1.wA + v2.a * v2.wA + v3.a * v3.wA;
                // *pB = *pA;
                break;

            default:
                assert (false);
                break;
            }
        }

        // djm pooled, from above
        public float getMetric()
        {
            switch (count)
            {
            case 1:
                return 0.0f;

            case 2:
                return MathUtils.distance(v1.w, v2.w);

            case 3:
                case3.set(v2.w).subLocal(v1.w);
                case33.set(v3.w).subLocal(v1.w);
                // return Vec2.cross(v2.w - v1.w, v3.w - v1.w);
                return Vec2.cross(case3, case33);

            default:
                assert (false);
                return 0.0f;
            }
        }

        // djm pooled from above
        /**
         * Solve a line segment using barycentric coordinates.
         */
        public void solve2()
        {
            // Solve a line segment using barycentric coordinates.
            //
            // p = a1 * w1 + a2 * w2
            // a1 + a2 = 1
            //
            // The vector from the origin to the closest point on the line is
            // perpendicular to the line.
            // e12 = w2 - w1
            // dot(p, e) = 0
            // a1 * dot(w1, e) + a2 * dot(w2, e) = 0
            //
            // 2-by-2 linear system
            // [1 1 ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            //
            // Define
            // d12_1 = dot(w2, e12)
            // d12_2 = -dot(w1, e12)
            // d12 = d12_1 + d12_2
            //
            // Solution
            // a1 = d12_1 / d12
            // a2 = d12_2 / d12
            final Vec2 w1 = v1.w;
            final Vec2 w2 = v2.w;
            e12.set(w2).subLocal(w1);
            // w1 region
            float d12_2 = -Vec2.dot(w1, e12);
            if (d12_2 <= 0.0f)
            {
                // a2 <= 0, so we clamp it to 0
                v1.a = 1.0f;
                count = 1;
                return;
            }
            // w2 region
            float d12_1 = Vec2.dot(w2, e12);
            if (d12_1 <= 0.0f)
            {
                // a1 <= 0, so we clamp it to 0
                v2.a = 1.0f;
                count = 1;
                v1.set(v2);
                return;
            }
            // Must be in e12 region.
            float inv_d12 = 1.0f / (d12_1 + d12_2);
            v1.a = d12_1 * inv_d12;
            v2.a = d12_2 * inv_d12;
            count = 2;
        }

        // djm pooled, and from above
        private final Vec2 e13 = new Vec2();

        private final Vec2 e23 = new Vec2();

        private final Vec2 w1 = new Vec2();

        private final Vec2 w2 = new Vec2();

        private final Vec2 w3 = new Vec2();

        /**
         * Solve a line segment using barycentric coordinates.<br/>
         * Possible regions:<br/>
         * - points[2]<br/>
         * - edge points[0]-points[2]<br/>
         * - edge points[1]-points[2]<br/>
         * - inside the triangle
         */
        public void solve3()
        {
            w1.set(v1.w);
            w2.set(v2.w);
            w3.set(v3.w);
            // Edge12
            // [1 1 ][a1] = [1]
            // [w1.e12 w2.e12][a2] = [0]
            // a3 = 0
            e12.set(w2).subLocal(w1);
            float w1e12 = Vec2.dot(w1, e12);
            float d12_1 = Vec2.dot(w2, e12);
            float d12_2 = -w1e12;
            // Edge13
            // [1 1 ][a1] = [1]
            // [w1.e13 w3.e13][a3] = [0]
            // a2 = 0
            e13.set(w3).subLocal(w1);
            float w1e13 = Vec2.dot(w1, e13);
            float d13_1 = Vec2.dot(w3, e13);
            float d13_2 = -w1e13;
            // Edge23
            // [1 1 ][a2] = [1]
            // [w2.e23 w3.e23][a3] = [0]
            // a1 = 0
            e23.set(w3).subLocal(w2);
            float w2e23 = Vec2.dot(w2, e23);
            float d23_1 = Vec2.dot(w3, e23);
            float d23_2 = -w2e23;
            // Triangle123
            float n123 = Vec2.cross(e12, e13);
            float d123_1 = n123 * Vec2.cross(w2, w3);
            float d123_2 = n123 * Vec2.cross(w3, w1);
            float d123_3 = n123 * Vec2.cross(w1, w2);
            // w1 region
            if (d12_2 <= 0.0f && d13_2 <= 0.0f)
            {
                v1.a = 1.0f;
                count = 1;
                return;
            }
            // e12
            if (d12_1 > 0.0f && d12_2 > 0.0f && d123_3 <= 0.0f)
            {
                float inv_d12 = 1.0f / (d12_1 + d12_2);
                v1.a = d12_1 * inv_d12;
                v2.a = d12_2 * inv_d12;
                count = 2;
                return;
            }
            // e13
            if (d13_1 > 0.0f && d13_2 > 0.0f && d123_2 <= 0.0f)
            {
                float inv_d13 = 1.0f / (d13_1 + d13_2);
                v1.a = d13_1 * inv_d13;
                v3.a = d13_2 * inv_d13;
                count = 2;
                v2.set(v3);
                return;
            }
            // w2 region
            if (d12_1 <= 0.0f && d23_2 <= 0.0f)
            {
                v2.a = 1.0f;
                count = 1;
                v1.set(v2);
                return;
            }
            // w3 region
            if (d13_1 <= 0.0f && d23_1 <= 0.0f)
            {
                v3.a = 1.0f;
                count = 1;
                v1.set(v3);
                return;
            }
            // e23
            if (d23_1 > 0.0f && d23_2 > 0.0f && d123_1 <= 0.0f)
            {
                float inv_d23 = 1.0f / (d23_1 + d23_2);
                v2.a = d23_1 * inv_d23;
                v3.a = d23_2 * inv_d23;
                count = 2;
                v1.set(v3);
                return;
            }
            // Must be in triangle123
            float inv_d123 = 1.0f / (d123_1 + d123_2 + d123_3);
            v1.a = d123_1 * inv_d123;
            v2.a = d123_2 * inv_d123;
            v3.a = d123_3 * inv_d123;
            count = 3;
        }
    }

    /**
     * A distance proxy is used by the GJK algorithm. It encapsulates any shape.
     * TODO: see if we can just do assignments with vertices, instead of copying
     * stuff over
     *
     * @author Daniel Murphy
     */
    public static class DistanceProxy
    {
        public final Vec2[] vertices;

        public int count;

        public float radius;

        public final Vec2[] buffer;

        public DistanceProxy()
        {
            vertices = new Vec2[Settings.maxPolygonVertices];
            for (int i = 0; i < vertices.length; i++)
            {
                vertices[i] = new Vec2();
            }
            buffer = new Vec2[2];
            count = 0;
            radius = 0f;
        }

        /**
         * Initialize the proxy using the given shape. The shape must remain in
         * scope while the proxy is in use.
         */
        public final void set(final Shape shape, int index)
        {
            switch (shape.getType())
            {
            case CIRCLE:
                final CircleShape circle = (CircleShape) shape;
                vertices[0].set(circle.p);
                count = 1;
                radius = circle.radius;
                break;

            case POLYGON:
                final PolygonShape poly = (PolygonShape) shape;
                count = poly.count;
                radius = poly.radius;
                for (int i = 0; i < count; i++)
                {
                    vertices[i].set(poly.vertices[i]);
                }
                break;

            case CHAIN:
                final ChainShape chain = (ChainShape) shape;
                assert (0 <= index && index < chain.count);
                buffer[0] = chain.vertices[index];
                if (index + 1 < chain.count)
                {
                    buffer[1] = chain.vertices[index + 1];
                }
                else
                {
                    buffer[1] = chain.vertices[0];
                }
                vertices[0].set(buffer[0]);
                vertices[1].set(buffer[1]);
                count = 2;
                radius = chain.radius;
                break;

            case EDGE:
                EdgeShape edge = (EdgeShape) shape;
                vertices[0].set(edge.vertex1);
                vertices[1].set(edge.vertex2);
                count = 2;
                radius = edge.radius;
                break;

            default:
                assert (false);
            }
        }

        /**
         * Get the supporting vertex index in the given direction.
         */
        public final int getSupport(final Vec2 d)
        {
            int bestIndex = 0;
            float bestValue = Vec2.dot(vertices[0], d);
            for (int i = 1; i < count; i++)
            {
                float value = Vec2.dot(vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }
            return bestIndex;
        }

        /**
         * Get the supporting vertex in the given direction.
         */
        public final Vec2 getSupportVertex(final Vec2 d)
        {
            int bestIndex = 0;
            float bestValue = Vec2.dot(vertices[0], d);
            for (int i = 1; i < count; i++)
            {
                float value = Vec2.dot(vertices[i], d);
                if (value > bestValue)
                {
                    bestIndex = i;
                    bestValue = value;
                }
            }
            return vertices[bestIndex];
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
         * Get a vertex by index. Used by Distance.
         */
        public final Vec2 getVertex(int index)
        {
            assert (0 <= index && index < count);
            return vertices[index];
        }
    }

    private final Simplex simplex = new Simplex();

    private final int[] saveA = new int[3];

    private final int[] saveB = new int[3];

    private final Vec2 closestPoint = new Vec2();

    private final Vec2 d = new Vec2();

    private final Vec2 temp = new Vec2();

    private final Vec2 normal = new Vec2();

    /**
     * Compute the closest points between two shapes. Supports any combination
     * of: CircleShape and PolygonShape. The simplex cache is input/output. On
     * the first call set SimplexCache.count to zero.
     */
    public final void distance(final DistanceOutput output,
            final SimplexCache cache, final DistanceInput input)
    {
        GJK_CALLS++;
        final DistanceProxy proxyA = input.proxyA;
        final DistanceProxy proxyB = input.proxyB;
        Transform transformA = input.transformA;
        Transform transformB = input.transformB;
        // Initialize the simplex.
        simplex.readCache(cache, proxyA, transformA, proxyB, transformB);
        // Get simplex vertices as an array.
        SimplexVertex[] vertices = simplex.vertices;
        // These store the vertices of the last simplex so that we
        // can check for duplicates and prevent cycling.
        // (pooled above)
        int saveCount;
        simplex.getClosestPoint(closestPoint);
        float distanceSqr1 = closestPoint.lengthSquared();
        float distanceSqr2 = distanceSqr1;
        // Main iteration loop
        int iter = 0;
        while (iter < MAX_ITERS)
        {
            // Copy simplex so we can identify duplicates.
            saveCount = simplex.count;
            for (int i = 0; i < saveCount; i++)
            {
                saveA[i] = vertices[i].indexA;
                saveB[i] = vertices[i].indexB;
            }
            switch (simplex.count)
            {
            case 1:
                break;

            case 2:
                simplex.solve2();
                break;

            case 3:
                simplex.solve3();
                break;

            default:
                assert (false);
            }
            // If we have 3 points, then the origin is in the corresponding
            // triangle.
            if (simplex.count == 3)
            {
                break;
            }
            // Compute closest point.
            simplex.getClosestPoint(closestPoint);
            distanceSqr2 = closestPoint.lengthSquared();
            // ensure progress
            // break;
            distanceSqr1 = distanceSqr2;
            // get search direction;
            simplex.getSearchDirection(d);
            // Ensure the search direction is numerically fit.
            if (d.lengthSquared() < Settings.EPSILON * Settings.EPSILON)
            {
                // The origin is probably contained by a line segment
                // or triangle. Thus the shapes are overlapped.
                // We can't return zero here even though there may be overlap.
                // In case the simplex is a point, segment, or triangle it is
                // difficult
                // to determine if the origin is contained in the CSO or very
                // close to it.
                break;
            }
            /*
             * SimplexVertex* vertex = vertices + simplex.count; vertex.indexA =
             * proxyA.GetSupport(MulT(transformA.R, -d)); vertex.wA =
             * Mul(transformA, proxyA.GetVertex(vertex.indexA)); Vec2 wBLocal;
             * vertex.indexB = proxyB.GetSupport(MulT(transformB.R, d));
             * vertex.wB = Mul(transformB, proxyB.GetVertex(vertex.indexB));
             * vertex.w = vertex.wB - vertex.wA;
             */
            // Compute a tentative new simplex vertex using support points.
            SimplexVertex vertex = vertices[simplex.count];
            Rot.mulTransUnsafe(transformA.q, d.negateLocal(), temp);
            vertex.indexA = proxyA.getSupport(temp);
            Transform.mulToOutUnsafe(transformA,
                    proxyA.getVertex(vertex.indexA), vertex.wA);
            // Vec2 wBLocal;
            Rot.mulTransUnsafe(transformB.q, d.negateLocal(), temp);
            vertex.indexB = proxyB.getSupport(temp);
            Transform.mulToOutUnsafe(transformB,
                    proxyB.getVertex(vertex.indexB), vertex.wB);
            vertex.w.set(vertex.wB).subLocal(vertex.wA);
            // Iteration count is equated to the number of support point calls.
            ++iter;
            ++GJK_ITERS;
            // Check for duplicate support points. This is the main termination
            // criteria.
            boolean duplicate = false;
            for (int i = 0; i < saveCount; ++i)
            {
                if (vertex.indexA == saveA[i] && vertex.indexB == saveB[i])
                {
                    duplicate = true;
                    break;
                }
            }
            // If we found a duplicate support point we must exit to avoid
            // cycling.
            if (duplicate)
            {
                break;
            }
            // New vertex is ok and needed.
            ++simplex.count;
        }
        GJK_MAX_ITERS = MathUtils.max(GJK_MAX_ITERS, iter);
        // Prepare output.
        simplex.getWitnessPoints(output.pointA, output.pointB);
        output.distance = MathUtils.distance(output.pointA, output.pointB);
        output.iterations = iter;
        // Cache the simplex.
        simplex.writeCache(cache);
        // Apply radii if requested.
        if (input.useRadii)
        {
            float rA = proxyA.radius;
            float rB = proxyB.radius;
            if (output.distance > rA + rB && output.distance > Settings.EPSILON)
            {
                // Shapes are still no overlapped.
                // Move the witness points to the outer surface.
                output.distance -= rA + rB;
                normal.set(output.pointB).subLocal(output.pointA);
                normal.normalize();
                temp.set(normal).mulLocal(rA);
                output.pointA.addLocal(temp);
                temp.set(normal).mulLocal(rB);
                output.pointB.subLocal(temp);
            }
            else
            {
                // Shapes are overlapped when radii are considered.
                // Move the witness points to the middle.
                // Vec2 p = 0.5f * (output.pointA + output.pointB);
                output.pointA.addLocal(output.pointB).mulLocal(.5f);
                output.pointB.set(output.pointA);
                output.distance = 0.0f;
            }
        }
    }
}
