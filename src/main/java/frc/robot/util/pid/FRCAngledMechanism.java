package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Robot;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.FRCSparkMax;

public class FRCAngledMechanism extends FRCMechanism {
    private final GearRatio gearRatio;

    @Override
    public double getRotation() {
        //return gearRatio.motorRotationsToDegrees(super.getRotation());
        //return (360 / (gearRatio.getLeadGear() / gearRatio.getFollowerAngle())) * super.getRotation();
        return gearRatio.getFollowerAngle(Rotation2d.fromRotations(super.getRotation())).getDegrees();
    }

    @Override
    public double getTargetRotation() {
        //return gearRatio.motorRotationsToDegrees(super.getTargetRotation());
        //return (360 / (gearRatio.getLeadGear() / gearRatio.getFollowerAngle())) * super.getTargetRotation();
        return gearRatio.getFollowerAngle(Rotation2d.fromRotations(super.getTargetRotation())).getDegrees();
    }

    @Override
    public FRCMechanism setTolerance(double angle) {
        super.setTolerance(angle);
        return this;
    }

    @Override
    public void setTarget(double angle) {
       // super.setTarget(((gearRatio.getLeadGear() / gearRatio.getFollowerAngle()) / 360) * angle);
        super.setTarget(gearRatio.getLeadAngle(Rotation2d.fromDegrees(angle)).getRotations());
    }

    public FRCAngledMechanism(String name, GearRatio ratio, FRCSparkMax motor, PIDConstants pidConstants) {
        super(name, motor, pidConstants);
        this.gearRatio = ratio;
    }
}