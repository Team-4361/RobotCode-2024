package frc.robot.util.swerve;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.*;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.config.ModuleSettings;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;

import static com.revrobotics.CANSparkBase.ControlType.kPosition;
import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.SWERVE_TUNING_ENABLED;

/**
 * A {@link SwerveModule} is composed of two motors and two encoders:
 * a drive motor/encoder and a turn motor/encoder. The turn motor is
 * responsible for controlling the direction the drive motor faces, essentially
 * allowing the robot to move in any direction.
 * <p></p>
 * NOTE: This system is <b>ONLY</b> designed to work with brushless motors!
 *
 * @author Eric Gold
 */
public class SwerveModule {
    private final SimpleMotorFeedforward driveFF;
    private final PIDController driveController;
    private final PIDController turnController;
    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;
    private final Rotation2d absOffset;
    private final StatusSignal<Double> signal;
    private final String name;

    private RelativeEncoder driveEncoder;
    private Double speedSetpoint = null;
    private Rotation2d angleSetpoint = null;

    private RelativeEncoder turnEncoder;
    private Rotation2d turnPosition;
    private Rotation2d turnAbsolutePosition;
    private double driveVelocityMPS;

    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name     The {@link String} name of the {@link SwerveModule}.
     * @param settings The {@link ModuleSettings} of the {@link SwerveModule}.
     */
    public SwerveModule(String name, ModuleSettings settings) {
        this.driveMotor = new FRCSparkMax(settings.getDriveID(), kBrushless, MotorModel.NEO);
        this.turnMotor = new FRCSparkMax(settings.getTurnID(), kBrushless, MotorModel.NEO);
        this.driveFF = new SimpleMotorFeedforward(MODULE_KS, MODULE_KV, MODULE_KA);

        this.name = name;
        this.driveController = PIDConstantsAK.generateController(DRIVE_PID);
        this.turnController = PIDConstantsAK.generateController(TURN_PID);
        this.absOffset = Rotation2d.fromDegrees(settings.getOffsetDegrees());
        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();

        turnEncoder.setPositionConversionFactor(TURN_POSITION_FACTOR);
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_FACTOR);

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);

        if (SWERVE_TUNING_ENABLED) {
            IOManager.initPIDTune("Swerve: Drive PID", driveController);
            IOManager.initPIDTune("Swerve: Turn PID", turnController);
        }

        try (CANcoder encoder = new CANcoder(settings.getEncoderID())) {
            CANcoderConfigurator cfg = encoder.getConfigurator();
            MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
            cfg.refresh(magnetSensorConfiguration);
            cfg.apply(magnetSensorConfiguration
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(settings.getOffsetDegrees() / 360));
            signal = encoder.getAbsolutePosition();
            BaseStatusSignal.setUpdateFrequencyForAll(50, signal);
            encoder.optimizeBusUtilization();
        }

        Timer.delay(1);
        update();
        turnEncoder.setPosition(turnAbsolutePosition.minus(absOffset).getDegrees());
    }

    public void update() {
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        ////////////////////////////////////////////// Update all the inputs inside this method.
        turnAbsolutePosition = Rotation2d.fromDegrees(signal.getValueAsDouble()*360).minus(absOffset);
        turnPosition = Rotation2d.fromDegrees(turnEncoder.getPosition());
        //////////////////////////////////////////////////////////////////////////////////////

        turnMotor.setVoltage(
                turnController.calculate(
                        turnPosition.getRadians(),
                        angleSetpoint.getRadians()
                )
        );
        if (speedSetpoint != null) {
            double adjustedSpeedMPS = speedSetpoint * Math.cos(turnController.getPositionError());
            driveMotor.setVoltage(
                    driveFF.calculate(adjustedSpeedMPS)
                        + driveController.calculate(driveVelocityMPS, adjustedSpeedMPS));
        }

    }

    public SwerveModuleState setState(SwerveModuleState desiredState) {
        desiredState = GlobalUtils.optimize(desiredState, getState().angle);

        // Prevent rotating module if speed is less than 1% to prevent jerky movement.
        angleSetpoint = (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_SPEED_MPS * 0.01))
                ? angleSetpoint
                : desiredState.angle;
        speedSetpoint = desiredState.speedMetersPerSecond;
        return desiredState;
    }

    /**
     * Get the {@link SwerveModuleState} based on the drive motor's velocity
     * (meters/sec) and the turn encoder's angle.
     *
     * @return a new {@link SwerveModuleState}, representing the module's
     * current state, based on the module's drive motor velocity (m/s)
     * and the turn encoder's angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                driveEncoder.getVelocity(),
                turnPosition
        );
    }


    /**
     * Get the {@link SwerveModulePosition} based on the drive motor's
     * distance travelled (in meters), and turn encoder's angle. This
     * is required for {@link SwerveDriveOdometry} to work correctly.
     *
     * @return A {@link SwerveModulePosition}, representing the module's
     * current position, based on the module's drive motor distance and
     * the turn encoder's angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                driveEncoder.getPosition(),
                turnAbsolutePosition
        );
    }

    /** @return The name of the {@link SwerveModule}. */
    public String getName() { return name; }
}