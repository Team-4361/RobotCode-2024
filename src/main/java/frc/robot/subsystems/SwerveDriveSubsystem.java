package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.pid.DashTunablePID;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.*;
import swervelib.parser.json.ModuleJson;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import static edu.wpi.first.wpilibj.Filesystem.getDeployDirectory;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.SWERVE_TUNING_ENABLED;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.LOW;

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
     * Initializes the {@link SwerveParser} and auto-generates all configuration information based on the files.
     * @return True if the operation was successful; false otherwise.
     */
    public static boolean initParser() {
        if (driveConfig != null && controllerConfig != null)
            return true; // already complete.
        try {
            new SwerveParser(new File(getDeployDirectory(), "swerve"));

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
        setCosineCompensator(!SwerveDriveTelemetry.isSimulation);
        SwerveDriveTelemetry.verbosity = SWERVE_TUNING_ENABLED ? HIGH : LOW;

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

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        /*
        SwerveModule flModule = swerveDrive.getModuleMap().get("frontleft");
        SwerveModule frModule = swerveDrive.getModuleMap().get("frontright");
        SwerveModule blModule = swerveDrive.getModuleMap().get("backleft");
        SwerveModule brModule = swerveDrive.getModuleMap().get("backright");

        if (driveTune != null && turnTune != null && System.currentTimeMillis() >= nextCheck) {
            PIDFConfig drive = new PIDFConfig(
                    driveTune.getP(),
                    driveTune.getI(),
                    driveTune.getD()
            );
            updateDrivePID(drive);

            PIDFConfig turn = new PIDFConfig(
                    turnTune.getP(),
                    turnTune.getI(),
                    turnTune.getD()
            );
            updateTurnPID(turn);

            driveTune.update();
            turnTune.update();
            nextCheck = System.currentTimeMillis() + 1500;
        }
         */
    }

    public void setStates(SwerveModuleState[] states) { this.setModuleStates(states, false); }

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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getOdometryHeading());

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
        this.zeroGyro();
        this.resetDriveEncoders();
        this.resetOdometry(pose);
    }

    public void reset() { reset(new Pose2d()); }
}
