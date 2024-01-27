package frc.robot.util.swerve.config;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
    @AutoLog
    class GyroIOInputs {
        public boolean connected = false;
        public Rotation2d yawPosition = new Rotation2d();
        public double yawVelocityRadPerSec = 0.0;
        public Rotation2d pitchPosition = new Rotation2d();
        public Rotation2d rollPosition = new Rotation2d();
        public boolean isCalibrating = false;
    }
    default void reset() {};

    default void updateInputs(GyroIOInputs inputs) {};
}
