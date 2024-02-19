package frc.robot.util.math;

import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import static edu.wpi.first.units.Units.Meters;

/**
 * This {@link PeakMotorDistance} class is designed to <b>easily</b> convert
 * Angular Motion. <--> Linear Motion. You must represent the <b>ABSOLUTE MAXIMUM</b>
 * values of your Mechanism; otherwise, it will not work correctly.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class PeakMotorDistance {
    private final Measure<Distance> maxDistance;
    private final double maxRotation;

    /**
     * Constructs a {@link PeakMotorDistance} instance.
     * @param distance The <b>MAXIMUM</b> {@link Distance} represented by <code>rotation</code>.
     * @param rotation The <b>MAXIMUM</b> rotations where the <code>distance</code> is reached.
     */
    public PeakMotorDistance(Measure<Distance> distance, double rotation) {
        assert rotation != 0;
        this.maxDistance = distance;
        this.maxRotation = rotation;
    }

    /** @return The peak {@link Distance} of the measurement. */
    public Measure<Distance> getDistance() {
        return maxDistance;
    }

    /** @return The Rotation for the measurement where {@link #getDistance()} is achieved. */
    public double getRotation() {
        return maxRotation;
    }

    /** @return rotation to distance based on unit */
    public double rotationToMeters(double currentRotation) {
        return (currentRotation / maxRotation) * maxDistance.in(Meters);
    }

    public double metersToRotation(double currentDistance) {
        return (currentDistance / maxDistance.in(Meters)) * maxRotation;
    }

}