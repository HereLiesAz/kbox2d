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
package com.hereliesaz.kbox2d.dynamics

import com.hereliesaz.kbox2d.common.Vec2

/**
 * A body definition holds all the data needed to construct a rigid body. You
 * can safely re-use body definitions. Shapes are added to a body after
 * construction.
 *
 * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L50-L125
 *
 * @author Daniel Murphy
 */
data class BodyDef(
    /**
     * The body type: static, kinematic, or dynamic. Note: if a dynamic body
     * would have zero mass, the mass is set to one.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L72-L74
     */
    var type: BodyType = BodyType.STATIC,
    /**
     * The world position of the body. Avoid creating bodies at the origin since
     * this can lead to many overlapping shapes.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L76-L78
     */
    var position: Vec2 = Vec2(),
    /**
     * The world angle of the body in radians.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L80-L81
     */
    var angle: Float = 0f,
    /**
     * The linear velocity of the body in world co-ordinates.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L83-L84
     */
    var linearVelocity: Vec2 = Vec2(),
    /**
     * The angular velocity of the body.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L86-L87
     */
    var angularVelocity: Float = 0f,
    /**
     * Linear damping is used to reduce the linear velocity. The damping
     * parameter can be larger than 1.0f but the damping effect becomes
     * sensitive to the time step when the damping parameter is large. Units are
     * `1 / time`.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L89-L93
     */
    var linearDamping: Float = 0f,
    /**
     * Angular damping is used to reduce the angular velocity. The damping
     * parameter can be larger than 1.0f but the damping effect becomes
     * sensitive to the time step when the damping parameter is large. Units are
     * `1 / time`.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L95-L99
     */
    var angularDamping: Float = 0f,
    /**
     * Set this flag to false if this body should never fall asleep. Note that
     * this increases CPU usage.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L101-L103
     */
    var allowSleep: Boolean = true,
    /**
     * Is this body initially awake or sleeping?
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L105-L106
     */
    var awake: Boolean = true,
    /**
     * Should this body be prevented from rotating? Useful for characters.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L108-L109
     */
    var fixedRotation: Boolean = false,
    /**
     * Is this a fast moving body that should be prevented from tunneling
     * through other moving bodies? Note that all bodies are prevented from
     * tunneling through kinematic and static bodies. This setting is only
     * considered on dynamic bodies.
     *
     * @warning You should use this flag sparingly since it increases processing
     * time.
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L111-L115
     */
    var bullet: Boolean = false,
    /**
     * Does this body start out active?
     *
     * TODO: rename to enabled
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L117-L118
     */
    var active: Boolean = true,
    /**
     * Use this to store application specific body data.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L120-L121
     */
    var userData: Any? = null,
    /**
     * Scale the gravity applied to this body.
     *
     * @repolink https://github.com/erincatto/box2d/blob/411acc32eb6d4f2e96fc70ddbdf01fe5f9b16230/include/box2d/b2_body.h#L123-L124
     */
    var gravityScale: Float = 1.0f
)
