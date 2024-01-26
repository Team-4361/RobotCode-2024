package frc.robot.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class SwerveModuleIOMAG extends SwerveModuleSparkBase {
    private final DutyCycleEncoder rotationEncoder;

    public SwerveModuleIOMAG(int driveId, int turnId, int dioPort) {
        super(driveId, turnId);
        this.rotationEncoder = new DutyCycleEncoder(dioPort);
    }

    @Override protected double getTurnRotation() { return rotationEncoder.get(); }
}

