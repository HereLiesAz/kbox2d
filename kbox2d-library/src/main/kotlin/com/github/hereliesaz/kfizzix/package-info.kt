/**
 * **jbox2d** is a 2D rigid body simulation library for games. Programmers
 * can use it in their games to make objects move in realistic ways and make the
 * game world more interactive. From the game engine's point of view, a physics
 * engine is just a system for procedural animation.
 *
 * ## Core Concepts
 *
 * Box2D works with several fundamental concepts and objects. We briefly define
 * these objects here and more details are given later in this document.
 *
 * ### shape
 *
 * A shape is 2D geometrical object, such as a circle or polygon.
 *
 * ### rigid body
 *
 * A chunk of matter that is so strong that the distance between any two bits of
 * matter on the chunk is constant. They are hard like a diamond. In the
 * following discussion we use body interchangeably with rigid body.
 *
 * ### fixture
 *
 * A fixture binds a shape to a body and adds material properties such as
 * density, friction, and restitution. A fixture puts a shape into the collision
 * system (broad-phase) so that it can collide with other shapes.
 *
 * ### constraint
 *
 * A constraint is a physical connection that removes degrees of freedom from
 * bodies. A 2D body has 3 degrees of freedom (two translation coordinates and
 * one rotation coordinate). If we take a body and pin it to the wall (like a
 * pendulum) we have constrained the body to the wall. At this point the body
 * can only rotate about the pin, so the constraint has removed 2 degrees of
 * freedom.
 *
 * ### contact constraint
 *
 * A special constraint designed to prevent penetration of rigid bodies and to
 * simulate friction and restitution. You do not create contact constraints;
 * they are created automatically by Box2D.
 *
 * ### joint
 *
 * This is a constraint used to hold two or more bodies together. Box2D supports
 * several joint types: revolute, prismatic, distance, and more. Some joints may
 * have limits and motors.
 *
 * ### joint limit
 *
 * A joint limit restricts the range of motion of a joint. For example, the
 * human elbow only allows a certain range of angles.
 *
 * ### joint motor
 *
 * A joint motor drives the motion of the connected bodies according to the
 * joint's degrees of freedom. For example, you can use a motor to drive the
 * rotation of an elbow.
 *
 * ### world
 *
 * A physics world is a collection of bodies, fixtures, and constraints that
 * interact together. Box2D supports the creation of multiple worlds, but this
 * is usually not necessary or desirable.
 *
 * ### solver
 *
 * The physics world has a solver that is used to advance time and to resolve
 * contact and joint constraints. The Box2D solver is a high performance
 * iterative solver that operates in order N time, where N is the number of
 * constraints.
 *
 * ### continuous collision
 *
 * The solver advances bodies in time using discrete time steps. Without
 * intervention this can lead to tunneling.
 *
 * ![tunneling1](https://raw.githubusercontent.com/engine-pi/jbox2d/main/misc/images/tunneling1.svg)
 *
 * Box2D contains specialized algorithms to deal with tunneling. First, the
 * collision algorithms can interpolate the motion of two bodies to find the
 * first time of impact (TOI). Second, there is a sub-stepping solver that moves
 * bodies to their first time of impact and then resolves the collision.
 *
 * ## Modules
 *
 * jbox2d is composed of three main modules:
 * [common],
 * [collision], and
 * [dynamics]. The Common module
 * has code for allocation, math, and settings. The Collision module defines
 * shapes, a broad-phase, and collision functions/queries. Finally the Dynamics
 * module provides the simulation world, bodies, fixtures, and joints.
 *
 * ![modules](https://raw.githubusercontent.com/engine-pi/jbox2d/main/misc/images/modules.svg)
 *
 * ## Units
 *
 * jbox2d works with floating point numbers and tolerances have to be used to
 * make jbox2d perform well. These tolerances have been tuned to work well with
 * meters-kilogram-second (MKS) units. In particular, jbox2d has been tuned to
 * work well with moving shapes between 0.1 and 10 meters. So this means objects
 * between soup cans and buses in size should work well. Static shapes may be up
 * to 50 meters long without trouble.
 *
 * Being a 2D physics engine, it is tempting to use pixels as your units.
 * Unfortunately this will lead to a poor simulation and possibly weird
 * behavior. An object of length 200 pixels would be seen by jbox2d as the size
 * of a 45 story building.
 *
 * > **Caution:** jbox2d is tuned for MKS units. Keep the size of moving objects
 * > roughly between 0.1 and 10 meters. You'll need to use some scaling system
 * > when you render your environment and actors. The jbox2d testbed does this by
 * > using an OpenGL viewport transform. DO NOT USE PIXELS.
 *
 * It is best to think of jbox2d bodies as moving billboards upon which you
 * attach your artwork. The billboard may move in a unit system of meters, but
 * you can convert that to pixel coordinates with a simple scaling factor. You
 * can then use those pixel coordinates to place your sprites, etc. You can also
 * account for flipped coordinate axes.
 *
 * Another limitation to consider is overall world size. If your world units
 * become larger than 2 kilometers or so, then the lost precision can affect
 * stability.
 *
 * > **Caution:** jbox2d works best with world sizes less than 2 kilometers.
 *
 * jbox2d uses radians for angles. The body rotation is stored in radians and
 * may grow unbounded. Consider normalizing the angle of your bodies if the
 * magnitude of the angle becomes too large
 * [Body.setTransform].
 *
 * > **Caution:** jbox2d uses radians, not degrees.
 *
 * Source:
 * [Box2D-Website](https://box2d.org/documentation/index.html)
 */
@file:JvmName("JBox2D")
package de.pirckheimer_gymnasium.jbox2d
