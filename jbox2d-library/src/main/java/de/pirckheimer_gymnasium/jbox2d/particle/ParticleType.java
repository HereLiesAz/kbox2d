package de.pirckheimer_gymnasium.jbox2d.particle;

/**
 * The particle type. Can be combined with | operator. Zero means liquid.
 *
 * @author Daniel Murphy
 */
public class ParticleType
{
    public static final int waterParticle = 0;

    /** removed after next step */
    public static final int zombieParticle = 1 << 1;

    /** zero velocity */
    public static final int wallParticle = 1 << 2;

    /** with restitution from stretching */
    public static final int springParticle = 1 << 3;

    /** with restitution from deformation */
    public static final int elasticParticle = 1 << 4;

    /** with viscosity */
    public static final int viscousParticle = 1 << 5;

    /** without isotropic pressure */
    public static final int powderParticle = 1 << 6;

    /** with surface tension */
    public static final int tensileParticle = 1 << 7;

    /** mixing color between contacting particles */
    public static final int colorMixingParticle = 1 << 8;

    /** call b2DestructionListener on destruction */
    public static final int destructionListener = 1 << 9;
}
