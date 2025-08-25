package de.chaffic.rays

import de.chaffic.geometry.bodies.TranslatableBody
import de.chaffic.math.Vec2

/**
 * A data class that stores information about a ray-body intersection.
 *
 * When a [Ray] successfully intersects with a body, it creates an instance of this class
 * to hold the details of the intersection.
 *
 * @property b The body that was hit by the ray.
 * @property coordinates The point of intersection in world coordinates.
 * @property index For polygon shapes, this is the index of the edge that was intersected. For other shapes, it is typically -1.
 *
 * @see Ray
 */
class RayInformation {
    val b: TranslatableBody
    val coordinates: Vec2
    val index: Int

    /**
     * Creates a new RayInformation object.
     *
     * @param b The body involved in the ray intersection.
     * @param x The x-coordinate of the intersection point.
     * @param y The y-coordinate of the intersection point.
     * @param index The index of the shape's side that the intersection occurred on.
     */
    constructor(b: TranslatableBody, x: Double, y: Double, index: Int) {
        this.b = b
        coordinates = Vec2(x, y)
        this.index = index
    }

    /**
     * Creates a new RayInformation object.
     *
     * @param b The body involved in the ray intersection.
     * @param v The position vector of the intersection point.
     * @param index The index of the shape's side that the intersection occurred on.
     */
    constructor(b: TranslatableBody, v: Vec2, index: Int) {
        this.b = b
        coordinates = v.copy()
        this.index = index
    }
}