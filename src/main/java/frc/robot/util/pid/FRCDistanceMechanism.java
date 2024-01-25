package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;

public class FRCDistanceMechanism extends FRCMechanism {
    private final PeakMotorDistance maximumDistance;

    /** @return The {@link PeakMotorDistance} used for this mechanism. */
    public PeakMotorDistance getMaximumDistance() { return this.maximumDistance; }

    @Override public double getRotation() { return maximumDistance.rotationToMeters(super.getRotation()); }

    @Override public double getTargetRotation() { return maximumDistance.rotationToMeters(super.getTargetRotation()); }

    @Override public void setTarget(double distance) { super.setTarget(maximumDistance.metersToRotation(distance)); }

    public FRCDistanceMechanism(String name, int motorID, IMotorModel model, PeakMotorDistance maxDistance, PIDConstants pidConstants) {
        super(name, motorID, model, pidConstants);
        this.maximumDistance = maxDistance;
        setForwardLimit(maximumDistance.getRotation());
    }
}