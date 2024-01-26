package frc.robot.util.swerve.config;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;
import frc.robot.util.motor.IMotorModel;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO extends IMotorModel {
    /**
     * This {@link SwerveModuleIOInputs} class has all inputs which will be recorded.
     */
    @AutoLog
    class SwerveModuleIOInputs {
        /** The turn rotations of the Swerve Module reported by the absolute encoder. */
        public double turnRotations;

        /** The drive rotations of the Swerve Module. */
        public double driveRotations;

        /** The drive velocity of the Swerve Motor. */
        public double driveRPM;

        /** The drive power of the Swerve Motor. */
        public double drivePower;

        /** The turn power of the Swerve Motor. */
        public double turnPower;
    }

    /** Resets all the encoders on the Swerve Module. */
    void reset();

    /**
     * Updates the inputs of the {@link SwerveModuleIO}.
     * @param inputs The {@link SwerveModuleIOInputs} to use.
     */
    void updateInputs(SwerveModuleIOInputs inputs);

    /** @return The {@link SparkPIDController} used for driving. */
    SparkPIDController getDrivePIDController();

    /**
     * Sets the integrated PID constants to the specific values.
     * @param pid The {@link PIDConstants} to use.
     */
    void setPID(PIDConstants pid);

    /**
     * Sets the drive power of the {@link SwerveModuleIO} in open-loop fashion.
     * @param power The power from -1.0 to +1.0
     */
    void driveOpenLoop(double power);

    /**
     * Sets the drive RPM of the {@link SwerveModuleIO} in closed-loop operation.
     * @param motorRPM The set-point drive motor RPM.
     */
    void driveClosedLoop(double motorRPM);

    /**
     * Sets the turn power of the {@link SwerveModuleIO} in open-loop fashion.
     * @param power The power from -1.0 to +1.0
     */
    void setTurnPower(double power);
}
