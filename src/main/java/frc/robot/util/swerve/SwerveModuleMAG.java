package frc.robot.util.swerve;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.util.swerve.config.ModuleSettings;

public class SwerveModuleMAG extends SwerveModuleBase {
    private final DutyCycleEncoder encoder;

    /**
     * Creates a new {@link SwerveModuleBase} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name     The {@link String} name of the {@link SwerveModuleBase}.
     * @param settings The {@link ModuleSettings} of the {@link SwerveModuleBase}.
     */
    public SwerveModuleMAG(String name, ModuleSettings settings) {
        super(name, settings);
        encoder = new DutyCycleEncoder(settings.getEncoderID());
    }

    @Override
    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition());
    }
}
