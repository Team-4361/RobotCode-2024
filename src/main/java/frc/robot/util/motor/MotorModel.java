package frc.robot.util.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Robot;

/**
 * This {@link MotorModel} class is designed to provide sample {@link IMotorModel} instances
 * which are used throughout the {@link Robot} code.
 *
 * @author Eric Gold
 */
public class MotorModel {
    public static final IMotorModel NEO = new IMotorModel() {
        @Override public int getMaximumStallCurrent() { return 105; }
        @Override public double getFreeSpeedRPM() { return 5676; }
        @Override public double getStallTorqueNM() { return 2.6;  }
        @Override public DCMotor getMotorInstance(int numMotors) { return DCMotor.getNEO(numMotors); }
    };
    public static final IMotorModel NEO_550 = new IMotorModel() {
        @Override public int getMaximumStallCurrent() { return 100; }
        @Override public double getFreeSpeedRPM() { return 11000; }
        @Override public double getStallTorqueNM() { return 0.97; }
        @Override public DCMotor getMotorInstance(int numMotors) { return DCMotor.getNeo550(numMotors); }
    };
}
