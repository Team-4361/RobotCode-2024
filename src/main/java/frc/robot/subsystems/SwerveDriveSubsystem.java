package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
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
import frc.robot.util.joystick.DriveHIDBase;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.parser.json.ModuleJson;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import static edu.wpi.first.wpilibj.Filesystem.getDeployDirectory;
import static frc.robot.Constants.Chassis.MAX_SPEED_MPS;
import static frc.robot.Constants.Chassis.SIDE_LENGTH_METERS;
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
    private static SwerveDriveConfiguration driveConfig = null;
    private static SwerveControllerConfiguration controllerConfig = null;


    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

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
     * Use PathPlanner Path finding to go to a point on the field.
     *
     * @param pose Target {@link Pose2d} to go to.
     * @return PathFinding command
     */
    public Command driveToPose(Pose2d pose) {
        PathConstraints constraints = new PathConstraints(
                getMaximumVelocity(), 4.0,
                getMaximumAngularVelocity(), Units.degreesToRadians(720));

        return AutoBuilder.pathfindToPose(
                pose,
                constraints,
                0.0,
                0.0
        );
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

            SwerveParser.physicalPropertiesJson.conversionFactor.drive = SwerveMath.calculateMetersPerRotation(
                    Units.inchesToMeters(4),
                    6.75,
                    1
            );
            SwerveParser.physicalPropertiesJson.conversionFactor.angle = SwerveMath.calculateDegreesPerSteeringRotation(
                    12.8,
                    1
            );

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
        setMotorIdleMode(true);
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
    public Pose2d getPose() {
        Pose2d current = super.getPose();
        Translation2d translation = current.getTranslation();
        return new Pose2d(new Translation2d(-translation.getX(), -translation.getY()), current.getRotation());
    }

    @Override
    public void periodic() {
        if (SWERVE_TUNING_ENABLED) {
            SmartDashboard.putNumber("FL Turn", getModuleMap().get("frontleft").getAbsolutePosition());
            SmartDashboard.putNumber("FR Turn", getModuleMap().get("frontright").getAbsolutePosition());
            SmartDashboard.putNumber("BL Turn", getModuleMap().get("backleft").getAbsolutePosition());
            SmartDashboard.putNumber("BR Turn", getModuleMap().get("backright").getAbsolutePosition());
            SmartDashboard.putString("Pose", getPose().toString());
        }
        
    }

    /**
     * Drives the Robot using one Joystick.
     * @param stick The {@link DriveHIDBase} to use.
     */
    public void drive(DriveHIDBase stick) {
        // Calculate the maximum speed based on XY and Twist.
        double xS = stick.getRobotX() * MAX_SPEED_MPS;
        double yS = stick.getRobotY() * MAX_SPEED_MPS;
        double tS = stick.getRobotTwist() * MAX_SPEED_MPS;

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getYaw());

        this.drive(speeds);
    }

    /**
     * Drives the Robot using two Joysticks (one for XY, one for Twist)
     * @param xyStick The {@link DriveHIDBase} to use for XY translation.
     * @param twistStick The {@link DriveHIDBase} to use for Twist.
     */
    public void drive(DriveHIDBase xyStick, DriveHIDBase twistStick) {
        double xS = xyStick.getRobotX() * MAX_SPEED_MPS;
        double yS = xyStick.getRobotY() * MAX_SPEED_MPS;
        double tS = twistStick.getRobotTwist() * MAX_SPEED_MPS;

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getOdometryHeading());

        this.drive(speeds);
    }

    public void reset(Pose2d pose) {
        this.resetDriveEncoders();
        this.resetOdometry(pose);
    }

    public void reset() { reset(new Pose2d()); }
}
