package de.chaffic.math;

import org.junit.Test;

import static junit.framework.TestCase.assertEquals;

public class Mat2Test {
    @Test
    public void setUsingRadians() {
        Mat2 m = new Mat2();
        m.set(1);
        assertEquals(m.getRow1().getX(), 0.5403023058681398);
        assertEquals(m.getRow2().getX(), 0.8414709848078965);
        assertEquals(m.getRow1().getY(), -0.8414709848078965);
        assertEquals(m.getRow2().getY(), 0.5403023058681398);
    }

    @Test
    public void setUsingMatrix() {
        Mat2 m = new Mat2();
        m.set(1);
        Mat2 u = new Mat2();
        u.set(m);
        assertEquals(u.getRow1().getX(), m.getRow1().getX());
        assertEquals(u.getRow2().getX(), m.getRow2().getX());
        assertEquals(u.getRow1().getY(), m.getRow1().getY());
        assertEquals(u.getRow2().getY(), m.getRow2().getY());
    }

    @Test
    public void transpose() {
        Mat2 m = new Mat2();
        m.set(1);
        Mat2 u = new Mat2();
        u.set(m);
        assertEquals(u.getRow1().getX(), m.getRow1().getX());
        assertEquals(u.getRow2().getX(), m.getRow2().getX());
        assertEquals(u.getRow1().getY(), m.getRow1().getY());
        assertEquals(u.getRow2().getY(), m.getRow2().getY());
    }

    @Test
    public void mul() {
        Mat2 m = new Mat2();
        m.set(1);
        Vec2 v = new Vec2(1, 0);
        m.mul(v);
        assertEquals(v.getX(), 0.5403023058681398);
        assertEquals(v.getY(), 0.8414709848078965);
    }

    @Test
    public void testMul() {
        Mat2 m = new Mat2();
        m.set(1);
        Vec2 v = new Vec2(1, 0);
        Vec2 q = new Vec2(10, 0);
        m.mul(v, q);
        assertEquals(q.getX(), 0.5403023058681398);
        assertEquals(q.getY(), 0.8414709848078965);
        assertEquals(v.getX(), 1.0);
        assertEquals(v.getY(), 0.0);
    }
}