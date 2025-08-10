package com.hereliesaz.kbox2d.common

/**
 * Orientated bounding box viewport transform
 *
 * @author Daniel Murphy
 */
class OBBViewportTransform : IViewportTransform {
    class OBB {
        val R = Mat22()
        val center = Vec2()
        val extents = Vec2()
    }

    protected val box = OBB()
    private var yFlip = false
    private val yFlipMat = Mat22(1f, 0f, 0f, -1f)

    init {
        box.R.setIdentity()
    }

    fun set(vpt: OBBViewportTransform) {
        box.center.set(vpt.box.center)
        box.extents.set(vpt.box.extents)
        box.R.set(vpt.box.R)
        yFlip = vpt.yFlip
    }

    fun setCamera(x: Float, y: Float, scale: Float) {
        box.center.set(x, y)
        Mat22.createScaleTransform(scale, box.R)
    }

    override val extents: Vec2
        get() = box.extents

    override val mat22Representation: Mat22
        get() = box.R

    fun setExtents(argExtents: Vec2) {
        box.extents.set(argExtents)
    }

    fun setExtents(halfWidth: Float, halfHeight: Float) {
        box.extents.set(halfWidth, halfHeight)
    }

    override val center: Vec2
        get() = box.center

    fun setCenter(argPos: Vec2) {
        box.center.set(argPos)
    }

    fun setCenter(x: Float, y: Float) {
        box.center.set(x, y)
    }

    /**
     * Gets the transform of the viewport, transforms around the center. Not a copy.
     */
    val transform: Mat22
        get() = box.R

    /**
     * Sets the transform of the viewport. Transforms about the center.
     */
    fun setTransform(transform: Mat22) {
        box.R.set(transform)
    }

    /**
     * Multiplies the obb transform by the given transform
     */
    override fun mulByTransform(transform: Mat22) {
        box.R.mulLocal(transform)
    }

    override fun isYFlip(): Boolean {
        return yFlip
    }



    override fun setYFlip(yFlip: Boolean) {
        this.yFlip = yFlip
    }

    private val inv = Mat22()
    override fun getScreenVectorToWorld(screen: Vec2, world: Vec2) {
        box.R.invertToOut(inv)
        inv.mulToOut(screen, world)
        if (yFlip) {
            yFlipMat.mulToOut(world, world)
        }
    }

    override fun getWorldVectorToScreen(world: Vec2, screen: Vec2) {
        box.R.mulToOut(world, screen)
        if (yFlip) {
            yFlipMat.mulToOut(screen, screen)
        }
    }

    override fun getWorldToScreen(world: Vec2, screen: Vec2) {
        screen.x = world.x - box.center.x
        screen.y = world.y - box.center.y
        box.R.mulToOut(screen, screen)
        if (yFlip) {
            yFlipMat.mulToOut(screen, screen)
        }
        screen.x += box.extents.x
        screen.y += box.extents.y
    }

    private val inv2 = Mat22()
    override fun getScreenToWorld(screen: Vec2, world: Vec2) {
        world.x = screen.x - box.extents.x
        world.y = screen.y - box.extents.y
        if (yFlip) {
            yFlipMat.mulToOut(world, world)
        }
        box.R.invertToOut(inv2)
        inv2.mulToOut(world, world)
        world.x += box.center.x
        world.y += box.center.y
    }
}
