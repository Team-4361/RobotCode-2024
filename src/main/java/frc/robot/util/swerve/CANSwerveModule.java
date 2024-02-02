package frc.robot.util.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.swerve.SwerveModule;
import frc.robot.util.swerve.config.ModuleSettings;

public class CANSwerveModule extends SwerveModule {
    private final StatusSignal<Double> turnAbsolutePosition;
    private final CANcoder cancoder;

    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name     The {@link String} name of the {@link SwerveModule}.
     * @param settings The {@link ModuleSettings} of the {@link SwerveModule}.
     */
    public CANSwerveModule(String name, ModuleSettings settings) {
        super(name, settings);
        cancoder = new CANcoder(settings.getEncoderID());
        cancoder.getConfigurator().apply(new CANcoderConfiguration());

        turnAbsolutePosition = cancoder.getAbsolutePosition();

        BaseStatusSignal.setUpdateFrequencyForAll(
                50.0,
                turnAbsolutePosition
        );
    }

    @Override
    public Rotation2d getRawAbsolutePosition() {
        BaseStatusSignal.refreshAll(turnAbsolutePosition);
        return Rotation2d.fromRotations(turnAbsolutePosition.getValueAsDouble());
    }
}
