package frc.robot.util.math;

import edu.wpi.first.math.geometry.Rotation2d;

/**
 * This {@link GearRatio} class enables operations with complex gear-ratios.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class GearRatio {
    private double gearOne;
    private double gearTwo;

    /** A direct drive {@link GearRatio} (1:1 ratio). */
    public static GearRatio DIRECT_DRIVE = GearRatio.from(1, 1);

    /**
     * Constructs a new {@link GearRatio} with a pre-determined Ratio.
     * @param ratio The {@link String}-equivalent ratio (ex. <code>1:5</code> or <code>0.5:2</code>)
     */
    public GearRatio(String ratio) {
        // attempt to split and convert the Ratio.
        try {
            int idx = ratio.indexOf(':');
            if (idx < 0)
                return;
            gearOne = Math.max(Double.parseDouble(ratio.substring(0, idx).trim()), 1);
            gearTwo = Math.max(Double.parseDouble(ratio.substring(idx + 1).trim()), 1);
        } catch (Exception ex) {
            gearOne = 1;
            gearTwo = 1;
        }
    }

    /**
     * Constructs a new {@link GearRatio} with a pre-determined Ratio.
     * @param gearOne The first gear teeth (driving gear)
     * @param gearTwo The second gear teeth (driven gear)
     */
    public GearRatio(double gearOne, double gearTwo) {
        this.gearOne = gearOne;
        this.gearTwo = gearTwo;
    }

    /**
     * Constructs a new {@link GearRatio} with a pre-determined Ratio.
     * @param gearOne The first gear teeth (driving gear)
     * @param gearTwo The second gear teeth (driven gear)
     */
    public static GearRatio from(double gearOne, double gearTwo) { return new GearRatio(gearOne, gearTwo); }

    /**
     * Constructs a new {@link GearRatio} with a pre-determined Ratio.
     * @param ratio The {@link String}-equivalent ratio (ex. <code>1:5</code> or <code>0.5:2</code>)
     */
    public static GearRatio from(String ratio) { return new GearRatio(ratio); }

    /** @return The first gear (driving gear) of the ratio. */
    public double getLeadGear() { return this.gearOne; }

    /** @return The second gear (driven gear) of the ratio. */
    public double getFollowerAngle() { return this.gearTwo; }

    /** @return The {@link Double} divisor (lead / follower) of the {@link GearRatio}. */
    public double getDivisor() { return gearOne / gearTwo; }

    /**
     * Combines two {@link GearRatio}s together by <b>multiplying</b> their ratios into a new instance.
     * @param other The {@link GearRatio} to combine.
     * @return A new combined {@link GearRatio}
     */
    public GearRatio add(GearRatio other) {
        double combOne = getLeadGear() * other.getLeadGear();
        double combTwo = getFollowerAngle() * other.getFollowerAngle();
        return new GearRatio(combOne, combTwo);
    }

    /**
     * @param leadRotations The number of rotations from the Leader Gear.
     * @return The {@link Rotation2d} angle.
     */
    public Rotation2d getFollowerAngle(Rotation2d leadRotations) {
        return Rotation2d.fromRotations(ExtendedMath.round(leadRotations.getRotations() * (gearTwo / gearOne)));
    }

    /**
     * @param leadRotations The number of rotations from the Leader Gear.
     * @return The number of rotations from the Follower Gear.
     */
    public double getFollowerRotations(double leadRotations) {
        return ExtendedMath.round(leadRotations * (gearTwo / gearOne));
    }

    /**
     * @param followerRotations The number of rotations of the Follower Gear.
     * @return The {@link Rotation2d} angle.
     */
    public Rotation2d getLeadAngle(Rotation2d followerRotations) {
        return Rotation2d.fromRotations(ExtendedMath.round(followerRotations.getRotations() * (gearOne / gearTwo)));
    }

    /**
     * @param followerRotations The number of rotations of the Follower Gear.
     * @return The number of rotations from the Leader Gear.
     */
    public double getLeadRotations(double followerRotations) {
        return ExtendedMath.round(followerRotations * (gearOne / gearTwo));
    }

    /**
     * @return The {@link GearRatio} in {@link String} format (ex. <code>1:5</code>)
     */
    @Override public String toString() { return gearOne + ":" + gearTwo; }
}
