package frc.robot.util.swerve;

import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveModuleIOMAG extends SwerveModuleSparkBase {
    private final DutyCycleEncoder absEncoder;

    @Override
    protected double getAbsoluteRotations() {
        return absEncoder.getAbsolutePosition();
    }

    public SwerveModuleIOMAG(int driveId, int turnId, int dioPort, double offsetRad) {
        super(driveId, turnId, offsetRad);
        this.absEncoder = new DutyCycleEncoder(dioPort);
    }
}