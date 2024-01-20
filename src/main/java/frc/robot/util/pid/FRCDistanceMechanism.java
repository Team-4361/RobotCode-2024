package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.motor.FRCSparkMax;

public class FRCDistanceMechanism extends FRCMechanism {
    private final PeakMotorDistance maximumDistance;

    @Override
    public double getRotation() {
        return maximumDistance.rotationToMeters(super.getRotation());
    }

    @Override
    public double getTargetRotation() {
        return maximumDistance.rotationToMeters(super.getTargetRotation());
    }

    @Override
    public void setTarget(double distance) {
        super.setTarget(maximumDistance.metersToRotation(distance));
    }

    public FRCDistanceMechanism(String name, FRCSparkMax motor, PeakMotorDistance maxDistance, PIDConstants pidConstants) {
        super(name, motor, pidConstants);
        this.maximumDistance = maxDistance;
        setForwardLimit(maximumDistance.getRotation());
    }
}