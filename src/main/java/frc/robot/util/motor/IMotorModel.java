package frc.robot.util.motor;

import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.Robot;

/**
 * This {@link IMotorModel} interface is exclusively used by {@link FRCSparkMax}, and provides a way to
 * increase the simulation accuracy of the {@link Robot}
 *
 * @author Eric Gold
 */
public interface IMotorModel {
    /** @return The maximum <b>theoretical</b> stall current of this {@link IMotorModel} in <b>amperes.</b> */
    int getMaximumStallCurrent();

    /** @return The maximum <b>theoretical</b> free speed of this {@link IMotorModel} in <b>RPM.</b> */
    double getFreeSpeedRPM();

    /** @return The maximum <b>theoretical</b> stall torque of this {@link IMotorModel} in <b>newton-meters.</b> */
    double getStallTorqueNM();

    /**
     * @param numMotors The number of motors to process.
     * @return The {@link DCMotor} instance used for simulation.
     */
    DCMotor getMotorInstance(int numMotors);
}
