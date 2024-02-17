package frc.robot.util.swerve;

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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunablePID;
import frc.robot.util.pid.PIDConstantsAK;

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
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private final PIDController turnController;
    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;
    private final Rotation2d absOffset;
    private final StatusSignal<Double> signal;
    private final String name;

    private Double speedSetpoint = null;
    private Rotation2d angleSetpoint = null;
    private boolean didSyncEncoders = false;
    private long nextSync = System.currentTimeMillis() + 5000;


    private RelativeEncoder turnEncoder;
    private RelativeEncoder driveEncoder;

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
        this.absOffset = Rotation2d.fromRotations(settings.getOffsetRotations());
        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();

        turnEncoder.setPositionConversionFactor(TURN_POSITION_FACTOR);
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_FACTOR);

        turnController.enableContinuousInput(-Math.PI, Math.PI);

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);

        if (SWERVE_TUNING_ENABLED) {
            // FIXME: change!
            this.driveTune = new DashTunablePID("Swerve: Drive PID", DRIVE_PID);
            this.turnTune = new DashTunablePID("Swerve: Turn PID", TURN_PID);

            driveTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
            turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);
        } else {
            this.driveTune = null;
            this.turnTune = null;
        }

        try (CANcoder encoder = new CANcoder(settings.getEncoderID())) {
            CANcoderConfigurator cfg = encoder.getConfigurator();
            MagnetSensorConfigs magnetSensorConfiguration = new MagnetSensorConfigs();
            cfg.refresh(magnetSensorConfiguration);
            cfg.apply(magnetSensorConfiguration
                    .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                    .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
                    .withMagnetOffset(settings.getOffsetRotations()));
            signal = encoder.getAbsolutePosition();
            //encoder.optimizeBusUtilization();
        }

        update();
    }

    public void synchronizeEncoders() {
        turnEncoder.setPosition(getAbsolutePosition().getDegrees());
    }

    public Rotation2d getAbsolutePosition() {
        return Rotation2d.fromDegrees((signal.getValueAsDouble()*360)).minus(absOffset);
    }

    public double getTurnDegrees() {
        return turnEncoder.getPosition();
    }

    public void update() {
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();
        signal.refresh();

        if (!didSyncEncoders && System.currentTimeMillis() >= nextSync) {
            synchronizeEncoders();
            didSyncEncoders = true;
        }

        if (driveTune != null)
            driveTune.update();
        if (turnTune != null)
            turnTune.update();

        if (angleSetpoint != null) {
            turnMotor.setVoltage(
                    turnController.calculate(
                            Units.degreesToRadians(getTurnDegrees()),
                            angleSetpoint.getRadians()
                    )
            );
            if (speedSetpoint != null) {
                double adjustedSpeedMPS = speedSetpoint * Math.cos(turnController.getPositionError());
                driveMotor.setVoltage(
                        driveFF.calculate(adjustedSpeedMPS)
                            + driveController.calculate(driveEncoder.getVelocity(), adjustedSpeedMPS));
            }
        }
        // TODO: dashboard!!!!!!!!!!
        String driveVelocity = name + ": rpm";
        String drivePower = name + ": pow";
        String turnPower = name + ": turn pow";
        String turnPosition = name + ": turn deg";
        String turnAbsPosition = name + ": abs turn deg";

        if (SWERVE_TUNING_ENABLED) {
            SmartDashboard.putNumber(driveVelocity, driveEncoder.getVelocity());
            SmartDashboard.putNumber(turnPower, turnMotor.get());
            SmartDashboard.putNumber(drivePower, driveMotor.get());
            SmartDashboard.putNumber(turnPosition, getTurnDegrees());
            SmartDashboard.putNumber(turnAbsPosition, getAbsolutePosition().getDegrees());
        }
    }

    public void setState(SwerveModuleState desiredState) {
        desiredState = GlobalUtils.optimize(desiredState, getState().angle);

        // Prevent rotating module if speed is less than 1% to prevent jerky movement.
        angleSetpoint = (Math.abs(desiredState.speedMetersPerSecond) <= (MAX_SPEED_MPS * 0.01))
                ? angleSetpoint
                : desiredState.angle;
        speedSetpoint = desiredState.speedMetersPerSecond;
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
                Rotation2d.fromDegrees(getTurnDegrees())
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
                getAbsolutePosition()
        );
    }

    /** @return The name of the {@link SwerveModule}. */
    public String getName() { return name; }
}