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
package de.pirckheimer_gymnasium.jbox2d.particle;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.Shape;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;

/**
 * A particle group definition holds all the data needed to construct a particle
 * group. You can safely re-use these definitions.
 */
public class ParticleGroupDef
{
    /** The particle-behavior flags. */
    public int flags;

    /** The group-construction flags. */
    public int groupFlags;

    /**
     * The world position of the group. Moves the group's shape a distance equal
     * to the value of position.
     */
    public final Vec2 position = new Vec2();

    /**
     * The world angle of the group in radians. Rotates the shape by an angle
     * equal to the value of angle.
     */
    public float angle;

    /** The linear velocity of the group's origin in world co-ordinates. */
    public final Vec2 linearVelocity = new Vec2();

    /** The angular velocity of the group. */
    public float angularVelocity;

    /** The color of all particles in the group. */
    public ParticleColor color;

    /**
     * The strength of cohesion among the particles in a group with flag
     * b2_elasticParticle or b2_springParticle.
     */
    public float strength;

    /** Shape containing the particle group. */
    public Shape shape;

    /**
     * If true, destroy the group automatically after its last particle has been
     * destroyed.
     */
    public boolean destroyAutomatically;

    /** Use this to store application-specific group data. */
    public Object userData;

    public ParticleGroupDef()
    {
        flags = 0;
        groupFlags = 0;
        angle = 0;
        angularVelocity = 0;
        strength = 1;
        destroyAutomatically = true;
    }
}
