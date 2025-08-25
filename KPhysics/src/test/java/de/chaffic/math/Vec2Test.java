package de.chaffic.math;

import org.junit.Test;

import static junit.framework.TestCase.assertEquals;
import static org.junit.Assert.*;

public class Vec2Test {
    @Test
    public void setUsingDoubleParameters() {
        Vec2 vec = new Vec2(4.0, 2.0);
        vec.set(33.0, 7.0);
        assertEquals(33.0, vec.getX(), 0);
        assertEquals(7.0, vec.getY(), 0);
    }

    @Test
    public void setToVector() {
        Vec2 vec = new Vec2(103.2, -2489273423.2);
        vec.set(new Vec2(-42.4, 92.1));
        assertEquals(vec.getX(), -42.4, 0);
        assertEquals(vec.getY(), 92.1, 0);
    }

    @Test
    public void copy() {
        Vec2 vec1 = new Vec2(1.0, 1.0);
        Vec2 vec2 = vec1.copy();
        assertNotEquals(vec1, vec1.copy());
        assertEquals(vec1.getX(), vec2.getX(), 0);
        assertEquals(vec1.getY(), vec2.getY(), 0);
    }

    @Test
    public void negative() {
        Vec2 vec = new Vec2(5.0, -7.0);
        vec.unaryMinus();
        assertEquals(vec.getX(), -5.0, 0);
        assertEquals(vec.getY(), 7.0, 0);

        Vec2 vec1 = vec.unaryMinus();
        assertEquals(vec1.getX(), 5.0, 0);
        assertEquals(vec1.getY(), -7.0, 0);
        assertEquals(vec.getX(), 5.0, 0);
        assertEquals(vec.getY(), -7.0, 0);
    }

    @Test
    public void negativeVec() {
        Vec2 vec1 = new Vec2(5.0, 1.0);
        Vec2 vec2 = vec1.copyNegative();
        assertEquals(5.0, vec1.getX(), 0);
        assertEquals(1.0, vec1.getY(), 0);
        assertEquals(-5.0, vec2.getX(), 0);
        assertEquals(-1.0, vec2.getY(), 0);
    }

    @Test
    public void add() {
        Vec2 vec1 = new Vec2(5.0, 2.0);
        Vec2 vec2 = new Vec2(7.0, 1.0);
        vec1.add(vec2);
        assertEquals(12.0, vec1.getX(), 0);
        assertEquals(3.0, vec1.getY(), 0);
    }

    @Test
    public void addi() {
        Vec2 vec1 = new Vec2(5.0, 2.0);
        Vec2 vec2 = new Vec2(7.0, 1.0);
        vec2 = vec1.plus(vec2);
        assertEquals(5.0, vec1.getX(), 0);
        assertEquals(2.0, vec1.getY(), 0);
        assertEquals(12.0, vec2.getX(), 0);
        assertEquals(3.0, vec2.getY(), 0);
    }

    @Test
    public void normal() {
        Vec2 vec1 = new Vec2(0.0, 1.0);
        Vec2 val = vec1.normal();
        assertEquals(-1.0, val.getX(), 0);
        assertEquals(0.0, val.getY(), 0);
    }

    @Test
    public void normalize() {
        Vec2 vec1 = new Vec2(-345.34, 745.0);
        vec1.normalize();
        assertEquals(vec1.length(), 1.0, 0);
        assertEquals(vec1.getX(), -0.4205573495355269, 0.0);
        assertEquals(vec1.getY(), 0.9072659564602061, 0.0);
    }

    @Test
    public void getNormalize() {
        Vec2 vec1 = new Vec2(1, 7);
        Vec2 val = vec1.getNormalized();
        assertEquals(0.1414213562373095, val.getX(), 0.0);
        assertEquals(0.9899494936611665, val.getY(), 0.0);

        assertEquals(1, vec1.getX(), 0.0);
        assertEquals(7, vec1.getY(), 0.0);
    }

    @Test
    public void distance() {
        Vec2 vec1 = new Vec2(5.0, 2.0);
        Vec2 vec2 = new Vec2(7.0, 1.0);
        double dist = vec1.distance(vec2);
        //square root of 5
        assertEquals(2.2361, dist, 0.0001);
    }

    @Test
    public void subtract() {
        Vec2 vec1 = new Vec2(5.0, 2.0);
        Vec2 vec2 = new Vec2(7.0, 1.0);
        vec1 = vec1.minus(vec2);
        assertEquals(-2.0, vec1.getX(), 0);
        assertEquals(1.0, vec1.getY(), 0);
    }

    @Test
    public void vectorCrossProduct() {
        Vec2 vec1 = new Vec2(2.0, 3.0);
        Vec2 vec2 = new Vec2(5.0, 6.0);
        double i = vec1.cross(vec2);
        assertEquals(-3.0, i, 0);
    }

    @Test
    public void scalarCrossProduct() {
        Vec2 vec1 = new Vec2(2.0, 3.0);
        Vec2 cross = vec1.cross(4.0);
        assertEquals(2.0, vec1.getX(), 0);
        assertEquals(3.0, vec1.getY(), 0);

        assertEquals(-12.0, cross.getX(), 0);
        assertEquals(8.0, cross.getY(), 0);
    }

    @Test
    public void scalar() {
        Vec2 vec1 = new Vec2(5.0, 2.0);
        Vec2 vec2 = vec1.scalar(4.0f);
        assertEquals(5.0, vec1.getX(), 0);
        assertEquals(2.0, vec1.getY(), 0);
        assertEquals(20.0, vec2.getX(), 0);
        assertEquals(8.0, vec2.getY(), 0);
    }

    @Test
    public void dotProduct() {
        Vec2 vec1 = new Vec2(5.0, 1.0);
        Vec2 vec2 = new Vec2(15.0, 10.0);
        double i = vec1.dot(vec2);
        assertEquals(85.0, i, 0);
    }

    @Test
    public void length() {
        Vec2 vec1 = new Vec2(0, 7);
        double val = vec1.length();
        assertEquals(7.0, val, 0);
    }

    @Test
    public void cross1() {
        Vec2 vec1 = new Vec2(5.03, 1.30);
        Vec2 w = Vec2.cross(6, vec1);
        assertEquals(7.8, w.getX(), 1e-15);
        assertEquals(-30.18, w.getY(), 1e-15);

        vec1 = new Vec2(-3.75, 9.34);
        w = Vec2.cross(1003.4, vec1);
        assertEquals(9371.756, w.getX(), 1e-15);
        assertEquals(3762.75, w.getY(), 1e-15);
    }

    @Test
    public void cross2() {
        Vec2 vec1 = new Vec2(5.03, 1.30);
        Vec2 w = Vec2.cross(6, vec1);
        assertEquals(-7.8, w.getX(), 1e-15);
        assertEquals(30.18, w.getY(), 1e-15);

        vec1 = new Vec2(-3.75, 9.34);
        w = Vec2.cross(1003.4, vec1);
        assertEquals(-9371.756, w.getX(), 1e-15);
        assertEquals(-3762.75, w.getY(), 1e-15);
    }

    @Test
    public void isValid() {
        assertTrue(new Vec2(-34234.234234324, -324954.5).isValid());
        assertTrue(new Vec2(32454543, 543543534.6).isValid());
        assertTrue(new Vec2(Double.MAX_VALUE, -324954.5).isValid());
        assertTrue(new Vec2(32454543, -Double.MAX_VALUE).isValid());
        assertFalse(new Vec2(32454543, Double.POSITIVE_INFINITY).isValid());
        assertFalse(new Vec2(32454543, -Double.POSITIVE_INFINITY).isValid());

        assertFalse(new Vec2(Double.POSITIVE_INFINITY, 234.534).isValid());
        assertFalse(new Vec2(-Double.POSITIVE_INFINITY, 234.534324).isValid());

        //Has to be double inputs not integer or doesn't detect NAN
        assertFalse(new Vec2(Double.NaN, 234.534).isValid());
        assertFalse(new Vec2(34255234.4, Double.NaN).isValid());
    }

    @Test
    public void isZero() {
        Vec2 v = new Vec2();

        assertTrue(v.isZero());

        v.set(1.0, 0.0);
        assertFalse(v.isZero());

        v.set(1.0, 1.0);
        assertFalse(v.isZero());

        v.set(0.0, 1.0);
        assertFalse(v.isZero());
    }

    @Test
    public void directionConstructor() {
        Vec2 v = new Vec2(2.0f,3.0f);
        v.add(new Vec2(-1.2, -5.4));
        assertEquals(0.8, v.getX(), 0);
        assertEquals(-2.4, v.getY(), 1e-15);
    }
}