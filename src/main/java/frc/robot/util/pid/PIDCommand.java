package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;

public class PIDCommand {
    private final double targetValue;

    private double maxSpeed;
    private double tolerance;
    private double kP, kI, kD;

    public PIDCommand(double targetValue) {
        this.targetValue = targetValue;
        this.kP = 0;
        this.kI = 0;
        this.kD = 0;
    }

    protected void fillValues(PIDMechanismBase mechanism) {
        if (maxSpeed == 0) maxSpeed = mechanism.getMaxSpeed();
        if (tolerance == 0) tolerance = mechanism.getTolerance();
    }

    public PIDCommand withMaxSpeed(double maxSpeed) {
        this.maxSpeed = maxSpeed;
        return this;
    }

    public PIDCommand withTolerance(double tolerance) {
        this.tolerance = tolerance;
        return this;
    }

    public PIDCommand withPID(PIDConstants constants) {
        return withPID(constants.kP, constants.kI, constants.kD);
    }

    public PIDCommand withPID(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        return this;
    }
}
