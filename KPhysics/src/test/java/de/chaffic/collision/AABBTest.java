package de.chaffic.collision;

import de.chaffic.math.Vec2;
import de.chaffic.dynamics.Body;
import de.chaffic.geometry.Circle;
import de.chaffic.math.Vec2;
import org.junit.Test;

import static junit.framework.TestCase.assertEquals;
import static junit.framework.TestCase.assertFalse;
import static junit.framework.TestCase.assertTrue;
import static org.junit.Assert.*;

public class AABBTest {
    @Test
    public void set() {
        AxisAlignedBoundingBox b = new AxisAlignedBoundingBox();
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(10, 10), new Vec2(20, 20));
        b.set(a);
        Vec2 val = new Vec2(10, 10);
        assertEquals(val.getX(), b.getMin().getX(), 0.0);
        assertEquals(val.getY(), b.getMin().getY(), 0.0);
        assertEquals(new Vec2(20, 20).getX(), b.getMax().getX(), 0.0);
        assertEquals(new Vec2(20, 20).getY(), b.getMax().getY(), 0.0);

    }

    @Test
    public void getMin() {
        AxisAlignedBoundingBox b = new AxisAlignedBoundingBox();
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(10, 10), new Vec2(20, 20));
        b.set(a);
        Vec2 val = new Vec2(10, 10);
        assertEquals(val.getX(), b.getMin().getX(), 0.0);
        assertEquals(val.getY(), b.getMin().getY(), 0.0);

    }

    @Test
    public void getMax() {
        AxisAlignedBoundingBox b = new AxisAlignedBoundingBox();
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(10, 10), new Vec2(20, 20));
        b.set(a);
        assertEquals(new Vec2(20, 20).getX(), b.getMax().getX(), 0.0);
        assertEquals(new Vec2(20, 20).getY(), b.getMax().getY(), 0.0);

    }

    @Test
    public void isValid() {
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(100, 100), new Vec2(300, 300));
        assertTrue(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, 0), new Vec2(0, 0));
        assertTrue(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(Double.POSITIVE_INFINITY, 0), new Vec2(300, 300));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, 0), new Vec2(Double.POSITIVE_INFINITY, 300));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, 0), new Vec2(1, Double.POSITIVE_INFINITY));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, Double.POSITIVE_INFINITY), new Vec2(1, 1));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, -Double.POSITIVE_INFINITY), new Vec2(1, 1));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, Double.NaN), new Vec2(1, 1));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(0, Double.NaN), new Vec2(Double.NaN, 1));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(-0, -0), new Vec2(-0, -0));
        assertTrue(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(-10, -10), new Vec2(-0, -0));
        assertTrue(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(-0, -0), new Vec2(-10, -10));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(-10, -0), new Vec2(-10, -10));
        assertFalse(a.isValid());
        a = new AxisAlignedBoundingBox(new Vec2(-10, -0), new Vec2(-10, -10));
        assertFalse(a.isValid());
    }

    @Test
    public void AABBOverLap() {
        //Corner overlaps - top right
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(100, 100), new Vec2(300, 300));
        AxisAlignedBoundingBox b = new AxisAlignedBoundingBox(new Vec2(200, 200), new Vec2(400, 400));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Corner overlaps - top left
        a = new AxisAlignedBoundingBox(new Vec2(0, 0), new Vec2(200, 200));
        b = new AxisAlignedBoundingBox(new Vec2(-100, 100), new Vec2(100, 300));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Corner overlaps - bottom left
        a = new AxisAlignedBoundingBox(new Vec2(0, 0), new Vec2(200, 200));
        b = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));


        //Corner overlaps - bottom right
        a = new AxisAlignedBoundingBox(new Vec2(0, 0), new Vec2(200, 200));
        b = new AxisAlignedBoundingBox(new Vec2(100, -100), new Vec2(300, 100));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Middle overlaps - middle left
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-150, -50), new Vec2(50, 50));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Middle overlaps - middle right
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(50, -50), new Vec2(150, 50));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Middle overlaps - middle
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-150, -50), new Vec2(150, 50));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Middle overlaps - top
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-80, -50), new Vec2(50, 150));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Middle overlaps - bottom
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-80, -150), new Vec2(50, 50));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Middle overlaps - bottom
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-80, -150), new Vec2(50, 150));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //With in
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-80, -50), new Vec2(50, 50));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 1
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-200, 200), new Vec2(100, 500));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 2
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-80, 200), new Vec2(50, 500));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 3
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(500, 200), new Vec2(570, 500));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 4
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-200, -50), new Vec2(-150, 50));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 6
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(200, -50), new Vec2(250, 50));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 7
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-200, -2000), new Vec2(-100, -500));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 8
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(-80, -800), new Vec2(50, -500));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));

        //Quadrant 9
        a = new AxisAlignedBoundingBox(new Vec2(-100, -100), new Vec2(100, 100));
        b = new AxisAlignedBoundingBox(new Vec2(500, -700), new Vec2(570, -500));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));
    }

    @Test
    public void testAABBOverLap() {
        AxisAlignedBoundingBox b = new AxisAlignedBoundingBox(new Vec2(100, 300), new Vec2(300, 100));
        Vec2 point = new Vec2(100, 100);
        assertTrue(b.aabbOverlap(point));
        //Checks if its inside
        point = new Vec2(150, 120);
        assertTrue(b.aabbOverlap(point));

        point = new Vec2(100, 100);
        assertTrue(b.aabbOverlap(point));
        point = new Vec2(100, 300);
        assertTrue(b.aabbOverlap(point));

        //Checks if its outside
        point = new Vec2(50, 100);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(50, 50);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(150, 50);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(350, 50);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(350, 200);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(350, 500);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(200, 500);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(50, 500);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(50, 200);
        assertFalse(b.aabbOverlap(point));

        point = new Vec2(100, 500);
        assertFalse(b.aabbOverlap(point));
        point = new Vec2(500, 100);
        assertFalse(b.aabbOverlap(point));
    }

    @Test
    public void copy() {
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(-10, 10), new Vec2(10, 10));
        AxisAlignedBoundingBox b = a.copy();

        assertNotSame(b,a);
        assertEquals(a.getMin().getX(), b.getMin().getX());
        assertEquals(a.getMin().getY(), b.getMin().getY());
        assertEquals(a.getMax().getX(), b.getMax().getX());
        assertEquals(a.getMax().getY(), b.getMax().getY());
    }

    @Test
    public void addOffset() {
        AxisAlignedBoundingBox a = new AxisAlignedBoundingBox(new Vec2(-10, 10), new Vec2(10, 10));
        a.addOffset(new Vec2(10, 10));
        assertEquals(a.getMin().getX(), 0.0);
        assertEquals(a.getMin().getY(), 20.0);
        assertEquals(a.getMax().getX(), 20.0);
        assertEquals(a.getMax().getY(), 20.0);
    }

    @Test
    public void BodyOverlap() {
        Body a = new Body(new Circle(20), 0, 0);
        Body b = new Body(new Circle(20), 0, 0);
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));
        a.getPosition().add(new Vec2(41, 0));
        assertFalse(AxisAlignedBoundingBox.aabbOverlap(a, b));
        a.getPosition().add(new Vec2(-6, 10));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));
        a.getPosition().add(new Vec2(-34, -38));
        assertTrue(AxisAlignedBoundingBox.aabbOverlap(a, b));
    }
}