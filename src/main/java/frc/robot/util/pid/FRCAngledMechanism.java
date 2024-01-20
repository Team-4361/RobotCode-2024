package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.FRCSparkMax;

public class FRCAngledMechanism extends FRCMechanism {
    private final GearRatio gearRatio;

    @Override
    public double getRotation() {
        //return gearRatio.motorRotationsToDegrees(super.getRotation());
        return (360 / (gearRatio.getLeadGear() / gearRatio.getFollower())) * super.getRotation();
    }

    @Override
    public double getTargetRotation() {
        //return gearRatio.motorRotationsToDegrees(super.getTargetRotation());
        return (360 / (gearRatio.getLeadGear() / gearRatio.getFollower())) * super.getTargetRotation();
    }

    @Override
    public FRCMechanism setTolerance(double angle) {
        super.setTolerance(angle);
        return this;
    }

    @Override
    public void setTarget(double angle) {
        super.setTarget(((gearRatio.getLeadGear() / gearRatio.getFollower()) / 360) * angle);
    }

    public FRCAngledMechanism(String name, GearRatio ratio, FRCSparkMax motor, PIDConstants pidConstants) {
        super(name, motor, pidConstants);
        this.gearRatio = ratio;
    }
}