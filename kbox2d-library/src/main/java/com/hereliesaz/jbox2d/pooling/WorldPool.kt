package de.pirckheimer_gymnasium.jbox2d.pooling

import de.pirckheimer_gymnasium.jbox2d.collision.AABB
import de.pirckheimer_gymnasium.jbox2d.collision.Collision
import de.pirckheimer_gymnasium.jbox2d.collision.Distance
import de.pirckheimer_gymnasium.jbox2d.collision.TimeOfImpact
import de.pirckheimer_gymnasium.jbox2d.common.Mat22
import de.pirckheimer_gymnasium.jbox2d.common.Mat33
import de.pirckheimer_gymnasium.jbox2d.common.Rot
import de.pirckheimer_gymnasium.jbox2d.common.Vec2
import de.pirckheimer_gymnasium.jbox2d.common.Vec3
import de.pirckheimer_gymnasium.jbox2d.dynamics.contacts.Contact

/**
 * World pool interface
 *
 * @author Daniel Murphy
 */
interface WorldPool {
    val polyContactStack: DynamicStack<Contact>
    val circleContactStack: DynamicStack<Contact>
    val polyCircleContactStack: DynamicStack<Contact>
    val edgeCircleContactStack: DynamicStack<Contact>
    val edgePolyContactStack: DynamicStack<Contact>
    val chainCircleContactStack: DynamicStack<Contact>
    val chainPolyContactStack: DynamicStack<Contact>
    val collision: Collision
    val timeOfImpact: TimeOfImpact
    val distance: Distance

    fun popVec2(): Vec2
    fun popVec2(num: Int): Array<Vec2>
    fun pushVec2(num: Int)

    fun popVec3(): Vec3
    fun popVec3(num: Int): Array<Vec3>
    fun pushVec3(num: Int)

    fun popMat22(): Mat22
    fun popMat22(num: Int): Array<Mat22>
    fun pushMat22(num: Int)

    fun popMat33(): Mat33
    fun pushMat33(num: Int)

    fun popAABB(): AABB
    fun popAABB(num: Int): Array<AABB>
    fun pushAABB(num: Int)

    fun popRot(): Rot
    fun pushRot(num: Int)

    fun getFloatArray(argLength: Int): FloatArray
    fun getIntArray(argLength: Int): IntArray
    fun getVec2Array(argLength: Int): Array<Vec2>
}
