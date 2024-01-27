package frc.robot.util.swerve.config;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.motor.IMotorModel;
import org.littletonrobotics.junction.AutoLog;

public interface SwerveModuleIO extends IMotorModel {
    /**
     * This {@link SwerveModuleIOInputs} class has all inputs which will be recorded.
     */
    @AutoLog
    class SwerveModuleIOInputs {
        public double drivePositionRad = 0.0;
        public double driveVelocityRadPerSec = 0.0;
        public double driveAppliedVolts = 0.0;
        public double driveCurrentAmps = 0.0;
        public Rotation2d turnAbsolutePosition = new Rotation2d();
        public Rotation2d turnPosition = new Rotation2d();
        public double turnVelocityRadPerSec = 0.0;
        public double turnCurrentAmps = 0.0;
        public double turnAppliedVolts = 0.0;
    }

    /** Resets all the encoders on the Swerve Module. */
    default void reset() {};

    /**
     * Updates the inputs of the {@link SwerveModuleIO}.
     * @param inputs The {@link SwerveModuleIOInputs} to use.
     */
    default void updateInputs(SwerveModuleIOInputs inputs) {};

    default void setDriveVoltage(double volts) {};

    default void setTurnVoltage(double volts) {};
}
