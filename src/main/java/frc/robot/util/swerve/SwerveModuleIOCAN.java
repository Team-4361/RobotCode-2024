package frc.robot.util.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public class SwerveModuleIOCAN extends SwerveModuleSparkBase {
    private final StatusSignal<Double> absEncoderSignal;

    @Override
    protected double getAbsoluteRotations() {
        return absEncoderSignal.refresh().getValueAsDouble();
    }

    public SwerveModuleIOCAN(int driveId, int turnId, int dioPort, double offsetRad) {
        super(driveId, turnId, offsetRad);

        CANcoderConfigurator config;
        try (CANcoder absEncoder = new CANcoder(dioPort)) {
            this.absEncoderSignal = absEncoder.getAbsolutePosition();
            config = absEncoder.getConfigurator();
        }

        config.apply(new CANcoderConfiguration());
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        config.refresh(magnetConfig);
        config.apply(magnetConfig
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }
}