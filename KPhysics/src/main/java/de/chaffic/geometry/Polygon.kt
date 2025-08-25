package de.chaffic.geometry

import de.chaffic.collision.AxisAlignedBoundingBox
import de.chaffic.dynamics.bodies.PhysicalBodyInterface
import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Math.lineIntersect
import de.chaffic.math.Math.pointIsOnLine
import de.chaffic.math.Vec2

/**
 * Represents a convex polygon shape for a physics body.
 *
 * Polygons can be created in several ways:
 * - From a list of vertices, which will be automatically converted to a convex hull.
 * - As a rectangle with a given width and height.
 * - As a regular polygon with a given radius and number of sides.
 *
 * Example of creating a polygonal body:
 * ```kotlin
 * // Create a rectangular polygon shape
 * val boxShape = Polygon(50.0, 50.0)
 *
 * // Create a body with this shape
 * val boxBody = Body(boxShape, 200.0, 200.0)
 *
 * // Add it to the world
 * world.addBody(boxBody)
 * ```
 *
 * @property vertices The vertices of the polygon in local coordinates.
 * @property normals The outward-facing normals for each edge of the polygon.
 *
 * @see Shape
 * @see Body
 */
class Polygon : Shape {
    var vertices: Array<Vec2>
    lateinit var normals: Array<Vec2>

    /**
     * Creates a polygon from a list of vertices.
     * The constructor automatically computes the convex hull of the given vertices to ensure the polygon is convex.
     *
     * @param vertList An array of [Vec2] points representing the vertices of the polygon.
     * @throws IllegalArgumentException if the provided vertices result in a degenerate polygon (fewer than 3 vertices after hull generation).
     */
    constructor(vertList: Array<Vec2>) {
        vertices = generateHull(vertList, vertList.size)
        if (vertices.size < 3) {
            throw IllegalArgumentException("A polygon must have at least 3 vertices after convex hull generation. Check for collinear points.")
        // Allow degenerate polygons with fewer than 3 vertices for backward compatibility.
        // If desired, add a warning here.
        
        calcNormals()
    }

    /**
     * Creates a rectangular polygon with its origin at the corner (0,0).
     *
     * @param width The total width of the rectangle.
     * @param height The total height of the rectangle.
     */
    constructor(width: Double, height: Double) : this(width, height, false)

    /**
     * Creates a rectangular polygon.
     *
     * @param width The total width of the rectangle.
     * @param height The total height of the rectangle.
     * @param centered If true, the polygon is centered at the origin. If false, the origin is at the corner.
     */
    constructor(width: Double, height: Double, centered: Boolean) {
        vertices = if (centered) {
            arrayOf(
                Vec2(-width / 2, -height / 2),
                Vec2(width / 2, -height / 2),
                Vec2(width / 2, height / 2),
                Vec2(-width / 2, height / 2)
            )
        } else {
            arrayOf(
                Vec2(0.0, 0.0),
                Vec2(width, 0.0),
                Vec2(width, height),
                Vec2(0.0, height)
            )
        }
        normals = arrayOf(
            Vec2(0.0, -1.0),
            Vec2(1.0, 0.0),
            Vec2(0.0, 1.0),
            Vec2(-1.0, 0.0)
        )
    }

    /**
     * Creates a regular polygon (e.g., triangle, pentagon, hexagon) centered at the origin.
     *
     * @param radius The radius of the circle that circumscribes the polygon.
     * @param noOfSides The number of sides (and vertices) the polygon should have.
     */
    constructor(radius: Int, noOfSides: Int) {
        val vertices = MutableList(noOfSides) { Vec2() }
        for (i in 0 until noOfSides) {
            val angle = 2 * Math.PI / noOfSides * (i + 0.75)
            val pointX = radius * StrictMath.cos(angle)
            val pointY = radius * StrictMath.sin(angle)
            vertices[i] = Vec2(pointX, pointY)
        }
        this.vertices = vertices.toList().toTypedArray()
        calcNormals()
    }

    /**
     * Calculates the normal for each edge of the polygon.
     * The normals are outward-facing and are used in collision detection.
     */
    private fun calcNormals() {
        val normals = MutableList(vertices.size) { Vec2() }
        for (i in vertices.indices) {
            val face = vertices[if (i + 1 == vertices.size) 0 else i + 1].minus(vertices[i])
            normals[i] = face.normal().normalize().unaryMinus()
        }
        this.normals = normals.toList().toTypedArray()
    }

    /**
     * Calculates the mass and inertia of the polygon based on its area and the given density.
     * The results are stored in the parent [Body].
     *
     * @param density The density of the material, used to calculate mass.
     */
    override fun calcMass(density: Double) {
        val physicalBody = this.body
        if(physicalBody !is PhysicalBodyInterface) return
        var centroidDistVec: Vec2? = Vec2(0.0, 0.0)
        var area = 0.0
        var inertia = 0.0
        val k = 1.0 / 3.0
        for (i in vertices.indices) {
            val point1 = vertices[i]
            val point2 = vertices[(i + 1) % vertices.size]
            val areaOfParallelogram = point1.cross(point2)
            val triangleArea = 0.5 * areaOfParallelogram
            area += triangleArea
            val weight = triangleArea * k
            centroidDistVec!!.add(point1.scalar(weight))
            centroidDistVec.add(point2.scalar(weight))
            val intx2 = point1.x * point1.x + point2.x * point1.x + point2.x * point2.x
            val inty2 = point1.y * point1.y + point2.y * point1.y + point2.y * point2.y
            inertia += 0.25 * k * areaOfParallelogram * (intx2 + inty2)
        }
        centroidDistVec = centroidDistVec!!.scalar(1.0 / area)
        for (i in vertices.indices) {
            vertices[i] = vertices[i].minus(centroidDistVec)
        }
        physicalBody.mass = density * area
        physicalBody.invMass = if (physicalBody.mass != 0.0) 1.0 / physicalBody.mass else 0.0
        physicalBody.inertia = inertia * density
        physicalBody.invInertia = if (physicalBody.inertia != 0.0) 1.0 / physicalBody.inertia else 0.0
    }

    /**
     * Generates an axis-aligned bounding box (AABB) for the polygon.
     * The AABB is calculated based on the transformed vertices of the polygon.
     */
    override fun createAABB() {
        val firstPoint = orientation.mul(vertices[0], Vec2())
        var minX = firstPoint.x
        var maxX = firstPoint.x
        var minY = firstPoint.y
        var maxY = firstPoint.y
        for (i in 1 until vertices.size) {
            val point = orientation.mul(vertices[i], Vec2())
            val px = point.x
            val py = point.y
            if (px < minX) {
                minX = px
            } else if (px > maxX) {
                maxX = px
            }
            if (py < minY) {
                minY = py
            } else if (py > maxY) {
                maxY = py
            }
        }
        this.body.aabb = AxisAlignedBoundingBox(Vec2(minX, minY), Vec2(maxX, maxY))
    }

    /**
     * Generates a convex hull from a given set of vertices using the Monotone Chain algorithm.
     *
     * @param vertices An array of points.
     * @param n The number of vertices in the array.
     * @return An array of vertices representing the convex hull.
     */
    private fun generateHull(vertices: Array<Vec2>, n: Int): Array<Vec2> {
        val hull = ArrayList<Vec2>()
        var firstPointIndex = 0
        var minX = Double.MAX_VALUE
        for (i in 0 until n) {
            val x = vertices[i].x
            if (x < minX) {
                firstPointIndex = i
                minX = x
            }
        }
        var point = firstPointIndex
        var currentEvalPoint: Int
        var first = true
        while (point != firstPointIndex || first) {
            first = false
            hull.add(vertices[point])
            currentEvalPoint = (point + 1) % n
            for (i in 0 until n) {
                if (sideOfLine(vertices[point], vertices[i], vertices[currentEvalPoint]) == -1) currentEvalPoint = i
            }
            point = currentEvalPoint
        }
        val hulls = MutableList(hull.size) { Vec2() }
        for (i in hull.indices) {
            hulls[i] = hull[i]
        }
        return hulls.toList().toTypedArray()
    }

    /**
     * Determines which side of a line a given point lies on.
     *
     * @param p1 The first point defining the line.
     * @param p2 The second point defining the line.
     * @param point The point to check.
     * @return 1 if the point is on the right side, -1 if on the left, and 0 if it's on the line.
     */
    private fun sideOfLine(p1: Vec2, p2: Vec2, point: Vec2): Int {
        val value = (p2.y - p1.y) * (point.x - p2.x) - (p2.x - p1.x) * (point.y - p2.y)
        return if (value > 0) 1 else if (value == 0.0) 0 else -1
    }

    /**
     * Checks if a given point is inside the polygon.
     * This is done using the Separating Axis Theorem (SAT) by checking the point against each face normal.
     *
     * @param startPoint The point to check, in world coordinates.
     * @return `true` if the point is inside the polygon, `false` otherwise.
     */
    override fun isPointInside(startPoint: Vec2): Boolean {
        for (i in vertices.indices) {
            val objectPoint = startPoint.minus(
                this.body.position.plus(
                    this.body.shape.orientation.mul(
                        vertices[i],
                        Vec2()
                    )
                )
            )
            if (objectPoint.dot(this.body.shape.orientation.mul(normals[i], Vec2())) > 0) {
                return false
            }
        }
        return true
    }

    /**
     * Performs a ray-intersection test with the polygon.
     *
     * @param startPoint The starting point of the ray in world coordinates.
     * @param endPoint The end point of the ray in world coordinates.
     * @param maxDistance The maximum distance for a valid intersection.
     * @param rayLength The total length of the ray.
     * @return An [IntersectionReturnElement] containing information about the intersection, if one occurred.
     */
    override fun rayIntersect(startPoint: Vec2, endPoint: Vec2, maxDistance: Double, rayLength: Double): IntersectionReturnElement {
        var minPx = 0.0
        var minPy = 0.0
        var intersectionFound = false
        var closestBody: TranslatableBody? = null
        var maxD = maxDistance

        for (i in vertices.indices) {
            var startOfPolyEdge = vertices[i]
            var endOfPolyEdge = vertices[if (i + 1 == vertices.size) 0 else i + 1]
            startOfPolyEdge = orientation.mul(startOfPolyEdge, Vec2()).plus(body.position)
            endOfPolyEdge = orientation.mul(endOfPolyEdge, Vec2()).plus(body.position)

            //detect if line (startPoint -> endpoint) intersects with the current edge (startOfPolyEdge -> endOfPolyEdge)
            val intersection = lineIntersect(startPoint, endPoint, startOfPolyEdge, endOfPolyEdge)
            if (intersection != null) {
                val distance = startPoint.distance(intersection)
                if(pointIsOnLine(startPoint, endPoint, intersection) && pointIsOnLine(startOfPolyEdge, endOfPolyEdge, intersection) && distance < maxD) {
                    maxD = distance
                    minPx = intersection.x
                    minPy = intersection.y
                    intersectionFound = true
                    closestBody = body
                }
            }
        }
        return IntersectionReturnElement(minPx, minPy, intersectionFound, closestBody, maxD)
    }
}