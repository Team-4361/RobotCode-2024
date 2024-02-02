package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.swerve.config.ModuleSettings;

public class MAGSwerveModule extends SwerveModule {
    private final DutyCycleEncoder encoder;

    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name     The {@link String} name of the {@link SwerveModule}.
     * @param settings The {@link ModuleSettings} of the {@link SwerveModule}.
     */
    public MAGSwerveModule(String name, ModuleSettings settings) {
        super(name, settings);
        this.encoder = new DutyCycleEncoder(settings.getEncoderID());
    }

    @Override
    public Rotation2d getRawAbsolutePosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition());
    }
}
