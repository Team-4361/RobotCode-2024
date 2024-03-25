package frc.robot.util.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

public enum MotorModel {
    NEO(DCMotor.getNEO(1)),
    NEO_550(DCMotor.getNeo550(1));

    private final DCMotor motor;

    MotorModel(DCMotor motor) {
        this.motor = motor;
    }

    public double getMaximumStallCurrent() { return motor.stallCurrentAmps; }
    public double getFreeSpeedRPM() { return Units.radiansPerSecondToRotationsPerMinute(motor.freeSpeedRadPerSec); }
    public double getStallTorqueNM() { return motor.stallTorqueNewtonMeters; }
    public DCMotor getMotorInstance() { return this.motor; }
}
