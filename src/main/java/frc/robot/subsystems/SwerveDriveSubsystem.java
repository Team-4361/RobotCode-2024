package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.ModuleJson;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.Alert;

import java.io.File;
import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.wpilibj.Filesystem.getDeployDirectory;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.SWERVE_TUNING_ENABLED;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.*;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveModule}s, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem extends SwerveDrive implements Subsystem, Sendable {
    public boolean fieldOriented = true;
    public boolean slowMode = false;

    private static SwerveDriveConfiguration driveConfig = null;
    private static SwerveControllerConfiguration controllerConfig = null;
    private Alert focDisabledAlert = new Alert("Swerve foc disabled!", Alert.AlertType.WARNING);


    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

    public Command toggleSlowModeCommand() {
        return Commands.runEnd(
                () -> Robot.swerve.slowMode = true,
                () -> Robot.swerve.slowMode = false
        );
    }

    public Command resetCommand() { return Commands.runOnce(this::reset); }

    /** Stops the {@link SwerveDriveSubsystem} from moving. */
    public void stop() {
        setStates(new SwerveModuleState[]
                {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0))
                });
    }

    /**
     * Initializes the {@link SwerveParser} and auto-generates all configuration information based on the files.
     * @return True if the operation was successful; false otherwise.
     */
    public static boolean initParser() {
        if (driveConfig != null && controllerConfig != null)
            return true; // already complete.
        try {
            new SwerveParser(new File(getDeployDirectory(), "swerve"));

            SwerveParser.physicalPropertiesJson.conversionFactor.drive = 0.047286787200699704;
            SwerveParser.physicalPropertiesJson.conversionFactor.angle = 28.125;
            /* 
            SwerveParser.physicalPropertiesJson.conversionFactor.drive = SwerveMath.calculateMetersPerRotation(
                    Units.inchesToMeters(4),
                    6.75,
                    1
            );
            SwerveParser.physicalPropertiesJson.conversionFactor.angle = SwerveMath.calculateDegreesPerSteeringRotation(
                    12.8,
                    1
            );
            */
            

            // Upon a successful initialization, create the drive instance using the static variables.
            SwerveModuleConfiguration[] moduleConfigurations =
                    new SwerveModuleConfiguration[SwerveParser.moduleJsons.length];

            for (int i = 0; i < moduleConfigurations.length; ++i) {
                ModuleJson module = SwerveParser.moduleJsons[i];
                moduleConfigurations[i] = module.createModuleConfiguration(
                        SwerveParser.pidfPropertiesJson.angle,
                        SwerveParser.pidfPropertiesJson.drive,
                        SwerveParser.physicalPropertiesJson.createPhysicalProperties(),
                        SwerveParser.swerveDriveJson.modules[i]);
            }

            driveConfig = new SwerveDriveConfiguration(
                    moduleConfigurations,
                    SwerveParser.swerveDriveJson.imu.createIMU(),
                    SwerveParser.swerveDriveJson.invertedIMU,
                    SwerveMath.createDriveFeedforward(
                            SwerveParser.physicalPropertiesJson.optimalVoltage,
                            MAX_SPEED_MPS,
                            SwerveParser.physicalPropertiesJson.wheelGripCoefficientOfFriction
                    ),
                    SwerveParser.physicalPropertiesJson.createPhysicalProperties());

            controllerConfig = SwerveParser.controllerPropertiesJson.createControllerConfiguration(
                    driveConfig,
                    MAX_SPEED_MPS
            );
            return true;
        } catch (IOException ex) {
            return false;
        }
    }

    /** Gets the name of this {@link Subsystem}. */
    @Override public String getName() { return SendableRegistry.getName(this); }

    /**
     * Initializes the {@link SendableBuilder} with information provided from this {@link Subsystem}.
     * @param builder The {@link SendableBuilder} which is provided.
     */
    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("Subsystem");

        builder.addBooleanProperty(".hasDefault", () -> getDefaultCommand() != null, null);
        builder.addStringProperty(
                ".default",
                () -> getDefaultCommand() != null ? getDefaultCommand().getName() : "none",
                null);
        builder.addBooleanProperty(".hasCommand", () -> getCurrentCommand() != null, null);
        builder.addStringProperty(
                ".command",
                () -> getCurrentCommand() != null ? getCurrentCommand().getName() : "none",
                null);
    }

    public ChassisSpeeds calculateSpeedsToPose(Pose2d currentPose, Pose2d desiredPose, boolean usePhoton) {
        PIDController driveController, turnController;

        if (usePhoton) {
            driveController = Robot.shooterCamera.getPhotonDriveController();
            turnController = Robot.shooterCamera.getPhotonTurnController();
        } else {
            driveController = Robot.shooterCamera.getDriveController();
            turnController = Robot.shooterCamera.getTurnController();
        }
        
        double mX = Robot.shooterCamera.getMaxDriveSpeed();
        double jX = MathUtil.clamp(driveController.calculate(currentPose.getX(), desiredPose.getX()), -mX, mX);
        double jY = MathUtil.clamp(driveController.calculate(currentPose.getY(), desiredPose.getY()), -mX, mX);
        double jO = MathUtil.clamp(
                turnController.calculate(
                        currentPose.getRotation().getRadians(),
                        desiredPose.getRotation().getRadians()
                ),
                -PHOTON_TURN_MAX_SPEED,
                PHOTON_TURN_MAX_SPEED
        );
        return new ChassisSpeeds(
                jX * MAX_SPEED_MPS,
                jY * MAX_SPEED_MPS,
                jO * MAX_SPEED_MPS
        );
    }

    public ChassisSpeeds calculateSpeedsToPose(Pose2d desiredPose, boolean usePhoton) {
       return calculateSpeedsToPose(getPose(), desiredPose, usePhoton);
    }

    /**
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules. The {@link #initParser()} method
     * <b>MUST</b> be called prior to calling this constructor!
     */
    public SwerveDriveSubsystem() {
        super(
                driveConfig,
                controllerConfig,
                MAX_SPEED_MPS
        );

        setHeadingCorrection(false);
        setCosineCompensator(false);
        setDriveMotorBrake(true);
       // setAngleMotorBrake(true);
        SwerveDriveTelemetry.verbosity = SWERVE_TUNING_ENABLED ? HIGH : NONE;

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::reset,
                this::getRobotVelocity,
                this::setChassisSpeeds,
                new HolonomicPathFollowerConfig(
                        MAX_SPEED_MPS,
                        Math.hypot(SIDE_LENGTH_METERS/2, SIDE_LENGTH_METERS/2),
                        new ReplanningConfig()
                ), () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );
    }

    public void setStates(SwerveModuleState[] states) { this.setModuleStates(states, false); }

    @Override
    public void periodic() {
        if (SWERVE_TUNING_ENABLED) {
            SmartDashboard.putNumber("FL Turn", getModuleMap().get("frontleft").getAbsolutePosition());
            SmartDashboard.putNumber("FR Turn", getModuleMap().get("frontright").getAbsolutePosition());
            SmartDashboard.putNumber("BL Turn", getModuleMap().get("backleft").getAbsolutePosition());
            SmartDashboard.putNumber("BR Turn", getModuleMap().get("backright").getAbsolutePosition());
            
        }
        SmartDashboard.putString("Pose", getPose().toString());
        focDisabledAlert.set(!fieldOriented);
    }

    public void reset(Pose2d pose) {
        this.zeroGyro();
        this.resetOdometry(pose);
        this.synchronizeModuleEncoders();
    }

    /**
     * Command to drive the robot using translative values and heading as angular velocity.
     *
     * @param translationX     Translation in the X direction. Cubed for smoother controls.
     * @param translationY     Translation in the Y direction. Cubed for smoother controls.
     * @param angularRotationX Angular velocity of the robot to set. Cubed for smoother controls.
     * @return Drive command.
     */
    public Command driveCommand(DoubleSupplier translationX,
                                DoubleSupplier translationY,
                                DoubleSupplier angularRotationX) {
        return run(() -> {
            // Make the robot move
            drive(
                new Translation2d(
                        Math.pow(translationX.getAsDouble(), 3) * getMaximumVelocity(),
                        Math.pow(translationY.getAsDouble(), 3) * getMaximumVelocity()
                ),
                Math.pow(angularRotationX.getAsDouble(), 3) * getMaximumAngularVelocity(),
                fieldOriented,
                false
            );
        });
    }

    public void reset() { reset(new Pose2d()); }
}
