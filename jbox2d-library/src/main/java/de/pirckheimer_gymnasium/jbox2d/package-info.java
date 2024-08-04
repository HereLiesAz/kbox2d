/**
 * <b>jbox2d</b> is a 2D rigid body simulation library for games. Programmers
 * can use it in their games to make objects move in realistic ways and make the
 * game world more interactive. From the game engine's point of view, a physics
 * engine is just a system for procedural animation.
 *
 * <h2>Core Concepts</h2>
 *
 * <p>
 * Box2D works with several fundamental concepts and objects. We briefly define
 * these objects here and more details are given later in this document.
 * </p>
 *
 * <h3>shape</h3>
 * <p>
 * A shape is 2D geometrical object, such as a circle or polygon.
 * </p>
 *
 * <h3>rigid body</h3>
 * <p>
 * A chunk of matter that is so strong that the distance between any two bits of
 * matter on the chunk is constant. They are hard like a diamond. In the
 * following discussion we use body interchangeably with rigid body.
 * </p>
 *
 * <h3>fixture</h3>
 * <p>
 * A fixture binds a shape to a body and adds material properties such as
 * density, friction, and restitution. A fixture puts a shape into the collision
 * system (broad-phase) so that it can collide with other shapes.
 * </p>
 *
 * <h3>constraint</h3>
 * <p>
 * A constraint is a physical connection that removes degrees of freedom from
 * bodies. A 2D body has 3 degrees of freedom (two translation coordinates and
 * one rotation coordinate). If we take a body and pin it to the wall (like a
 * pendulum) we have constrained the body to the wall. At this point the body
 * can only rotate about the pin, so the constraint has removed 2 degrees of
 * freedom.
 * </p>
 *
 * <h3>contact constraint</h3>
 * <p>
 * A special constraint designed to prevent penetration of rigid bodies and to
 * simulate friction and restitution. You do not create contact constraints;
 * they are created automatically by Box2D.
 * </p>
 *
 * <h3>joint</h3>
 * <p>
 * This is a constraint used to hold two or more bodies together. Box2D supports
 * several joint types: revolute, prismatic, distance, and more. Some joints may
 * have limits and motors.
 * </p>
 *
 * <h3>joint limit</h3>
 * <p>
 * A joint limit restricts the range of motion of a joint. For example, the
 * human elbow only allows a certain range of angles.
 * </p>
 *
 * <h3>joint motor</h3>
 * <p>
 * A joint motor drives the motion of the connected bodies according to the
 * joint's degrees of freedom. For example, you can use a motor to drive the
 * rotation of an elbow.
 * </p>
 *
 * <h3>world</h3>
 * <p>
 * A physics world is a collection of bodies, fixtures, and constraints that
 * interact together. Box2D supports the creation of multiple worlds, but this
 * is usually not necessary or desirable.
 * </p>
 *
 * <h3>solver</h3>
 * <p>
 * The physics world has a solver that is used to advance time and to resolve
 * contact and joint constraints. The Box2D solver is a high performance
 * iterative solver that operates in order N time, where N is the number of
 * constraints.
 * </p>
 *
 * <h3>continuous collision</h3>
 * <p>
 * The solver advances bodies in time using discrete time steps. Without
 * intervention this can lead to tunneling.
 * </p>
 *
 * <p>
 * Box2D contains specialized algorithms to deal with tunneling. First, the
 * collision algorithms can interpolate the motion of two bodies to find the
 * first time of impact (TOI). Second, there is a sub-stepping solver that moves
 * bodies to their first time of impact and then resolves the collision.
 * </p>
 */
package de.pirckheimer_gymnasium.jbox2d;
