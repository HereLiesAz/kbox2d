package de.pirckheimer_gymnasium.jbox2d.testbed.tests;

import de.pirckheimer_gymnasium.jbox2d.collision.shapes.CircleShape;
import de.pirckheimer_gymnasium.jbox2d.collision.shapes.PolygonShape;
import de.pirckheimer_gymnasium.jbox2d.common.Transform;
import de.pirckheimer_gymnasium.jbox2d.common.Vec2;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleColor;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleGroup;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleGroupDef;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleGroupType;
import de.pirckheimer_gymnasium.jbox2d.particle.ParticleType;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedSettings;
import de.pirckheimer_gymnasium.jbox2d.testbed.framework.TestbedTest;

public class DrawingParticles extends TestbedTest
{
    ParticleGroup m_lastGroup;

    boolean m_drawing;

    int m_particleFlags;

    int m_groupFlags;

    ParticleColor color = new ParticleColor();

    @Override
    public void initTest(boolean deserialized)
    {
        {
            {
                PolygonShape shape = new PolygonShape();
                Vec2[] vertices = new Vec2[] { new Vec2(-40, -20),
                        new Vec2(40, -20), new Vec2(40, 0), new Vec2(-40, 0) };
                shape.set(vertices, 4);
                getGroundBody().createFixture(shape, 0.0f);
            }
            {
                PolygonShape shape = new PolygonShape();
                Vec2[] vertices = new Vec2[] { new Vec2(-40, -20),
                        new Vec2(-20, -20), new Vec2(-20, 60),
                        new Vec2(-40, 60) };
                shape.set(vertices, 4);
                getGroundBody().createFixture(shape, 0.0f);
            }
            {
                PolygonShape shape = new PolygonShape();
                Vec2[] vertices = new Vec2[] { new Vec2(20, -20),
                        new Vec2(40, -20), new Vec2(40, 60), new Vec2(20, 60) };
                shape.set(vertices, 4);
                getGroundBody().createFixture(shape, 0.0f);
            }
            {
                PolygonShape shape = new PolygonShape();
                Vec2[] vertices = new Vec2[] { new Vec2(-40, 40),
                        new Vec2(40, 40), new Vec2(40, 60), new Vec2(-40, 60) };
                shape.set(vertices, 4);
                getGroundBody().createFixture(shape, 0.0f);
            }
        }
        world.setParticleRadius(0.5f);
        m_lastGroup = null;
        m_drawing = true;
        m_groupFlags = 0;
    }

    @Override
    public void step(TestbedSettings settings)
    {
        super.step(settings);
        addTextLine("Keys: (L) liquid, (E) elastic, (S) spring");
        addTextLine("(F) rigid, (W) wall, (V) viscous, (T) tensile");
        addTextLine("(O) powder (Z) erase, (X) move");
    }

    public void keyPressed(char keyChar, int keyCode)
    {
        m_drawing = keyChar != 'x';
        m_particleFlags = 0;
        m_groupFlags = 0;
        color.set((byte) 127, (byte) 127, (byte) 127, (byte) 50);
        switch (keyChar)
        {
        case 'e':
            m_particleFlags = ParticleType.elasticParticle;
            m_groupFlags = ParticleGroupType.solidParticleGroup;
            break;

        case 'o':
            m_particleFlags = ParticleType.powderParticle;
            break;

        case 'f':
            m_groupFlags = ParticleGroupType.rigidParticleGroup
                    | ParticleGroupType.solidParticleGroup;
            break;

        case 's':
            m_particleFlags = ParticleType.springParticle;
            m_groupFlags = ParticleGroupType.solidParticleGroup;
            break;

        case 't':
            color.set((byte) 0, (byte) 127, (byte) 0, (byte) 50);
            m_particleFlags = ParticleType.tensileParticle;
            break;

        case 'v':
            color.set((byte) 0, (byte) 0, (byte) 127, (byte) 50);
            m_particleFlags = ParticleType.viscousParticle;
            break;

        case 'w':
            m_particleFlags = ParticleType.wallParticle;
            m_groupFlags = ParticleGroupType.solidParticleGroup;
            break;

        case 'z':
            m_particleFlags = ParticleType.zombieParticle;
            break;
        }
    }

    Transform pxf = new Transform();

    CircleShape pshape = new CircleShape();

    ParticleGroupDef ppd = new ParticleGroupDef();

    @Override
    public void mouseDrag(Vec2 p, int button)
    {
        super.mouseDrag(p, button);
        if (m_drawing)
        {
            pshape.p.set(p);
            pshape.radius = 2.0f;
            pxf.setIdentity();
            world.destroyParticlesInShape(pshape, pxf);
            ppd.shape = pshape;
            ppd.color = color;
            ppd.flags = m_particleFlags;
            ppd.groupFlags = m_groupFlags;
            ParticleGroup group = world.createParticleGroup(ppd);
            if (m_lastGroup != null
                    && group.getGroupFlags() == m_lastGroup.getGroupFlags())
            {
                world.joinParticleGroups(m_lastGroup, group);
            }
            else
            {
                m_lastGroup = group;
            }
            mouseTracing = false;
        }
    }

    @Override
    public void mouseUp(Vec2 p, int button)
    {
        super.mouseUp(p, button);
        m_lastGroup = null;
    }

    @Override
    public void particleGroupDestroyed(ParticleGroup group)
    {
        super.particleGroupDestroyed(group);
        if (group == m_lastGroup)
        {
            m_lastGroup = null;
        }
    }

    @Override
    public String getTestName()
    {
        return "Drawing Particles";
    }
}
