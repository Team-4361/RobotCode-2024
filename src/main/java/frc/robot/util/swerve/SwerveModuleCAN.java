package frc.robot.util.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.swerve.config.ModuleSettings;

public class SwerveModuleCAN extends SwerveModuleBase {
    private final StatusSignal<Double> signal;

    /**
     * Creates a new {@link SwerveModuleBase} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name     The {@link String} name of the {@link SwerveModuleBase}.
     * @param settings The {@link ModuleSettings} of the {@link SwerveModuleBase}.
     */
    public SwerveModuleCAN(String name, ModuleSettings settings) {
        super(name, settings);
        CANcoderConfigurator config;
        try (CANcoder encoder = new CANcoder(settings.getEncoderID())) {
            signal = encoder.getAbsolutePosition();
            config = encoder.getConfigurator();
            config.apply(new CANcoderConfiguration());
            MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
            config.refresh(magnetConfig);
            config.apply(magnetConfig
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
        }
    }

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromRotations(signal.refresh().getValueAsDouble());
    }
}
