package frc.robot.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleIOCAN extends SwerveModuleSparkBase {
    private final CANcoder rotationEncoder;

    public SwerveModuleIOCAN(int driveId, int turnId, int dioPort) {
        super(driveId, turnId);
        this.rotationEncoder = new CANcoder(dioPort);
        CANcoderConfigurator config = rotationEncoder.getConfigurator();
        config.apply(new CANcoderConfiguration()); // reset

        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        config.refresh(magnetConfig);
        config.apply(magnetConfig
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    @Override
    protected double getTurnRotation() {
        return rotationEncoder
                .getPosition()
                .refresh()
                .getValueAsDouble();
    }
}
