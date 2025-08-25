package de.chaffic.geometry

import de.chaffic.math.Vec2
import org.junit.Assert.assertEquals
import org.junit.Test

class PolygonTest {

    @Test
    fun testCreateRectangleCorner() {
        val polygon = Polygon(100.0, 50.0)
        assertEquals(4, polygon.vertices.size)
        assertEquals(Vec2(0.0, 0.0), polygon.vertices[0])
        assertEquals(Vec2(100.0, 0.0), polygon.vertices[1])
        assertEquals(Vec2(100.0, 50.0), polygon.vertices[2])
        assertEquals(Vec2(0.0, 50.0), polygon.vertices[3])

        assertEquals(4, polygon.normals.size)
        assertEquals(Vec2(0.0, -1.0), polygon.normals[0])
        assertEquals(Vec2(1.0, 0.0), polygon.normals[1])
        assertEquals(Vec2(0.0, 1.0), polygon.normals[2])
        assertEquals(Vec2(-1.0, 0.0), polygon.normals[3])
    }

    @Test
    fun testCreateRectangleCentered() {
        val polygon = Polygon(100.0, 50.0, true)
        assertEquals(4, polygon.vertices.size)
        assertEquals(Vec2(-50.0, -25.0), polygon.vertices[0])
        assertEquals(Vec2(50.0, -25.0), polygon.vertices[1])
        assertEquals(Vec2(50.0, 25.0), polygon.vertices[2])
        assertEquals(Vec2(-50.0, 25.0), polygon.vertices[3])

        assertEquals(4, polygon.normals.size)
        assertEquals(Vec2(0.0, -1.0), polygon.normals[0])
        assertEquals(Vec2(1.0, 0.0), polygon.normals[1])
        assertEquals(Vec2(0.0, 1.0), polygon.normals[2])
        assertEquals(Vec2(-1.0, 0.0), polygon.normals[3])
    }
}
