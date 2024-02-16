package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.swerve.SwerveModule;

import java.util.Optional;

import static frc.robot.Constants.AlertConfig.STRING_GYRO_CALIBRATING;
import static frc.robot.Constants.AlertConfig.STRING_NO_GYRO;
import static frc.robot.Constants.Chassis.*;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveModule}s, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final AHRS gyro;
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveDrivePoseEstimator poseEstimator;

    public Rotation2d yawPosition = new Rotation2d();
    public boolean fieldOriented = true;

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
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules.
     */
    public SwerveDriveSubsystem() {
        this.modules = new SwerveModule[]{
                new SwerveModule("FL", FL_MODULE),
                new SwerveModule("FR", FR_MODULE),
                new SwerveModule("BL", BL_MODULE),
                new SwerveModule("BR", BR_MODULE),
        };

        this.gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

        // Define the distances relative to the center of the robot. +X is forward and +Y is left.
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(SIDE_LENGTH_METERS / 2, SIDE_LENGTH_METERS / 2),  // FL
                new Translation2d(SIDE_LENGTH_METERS / 2, -SIDE_LENGTH_METERS / 2), // FR
                new Translation2d(-SIDE_LENGTH_METERS / 2, SIDE_LENGTH_METERS / 2), // BL
                new Translation2d(-SIDE_LENGTH_METERS / 2, -SIDE_LENGTH_METERS / 2) // BR
        );

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

        IOManager.getAlert(STRING_GYRO_CALIBRATING, AlertType.WARNING)
                .setCondition(gyro::isCalibrating)
                .setPersistent(false)
                .setDisableDelay(2000)
                .setOneUse(false);

        IOManager.getAlert(STRING_NO_GYRO, AlertType.ERROR)
                .setCondition(() -> !gyro.isConnected())
                .setPersistent(true);


        AutoBuilder.configureHolonomic(
                this::getPose,
                this::reset,
                this::getRobotVelocity,
                this::setSpeeds,
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
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle
        // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                kinematics.toChassisSpeeds(getStates()), getHeading().unaryMinus());
    }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        yawPosition = gyro.getRotation2d();
        for (SwerveModule module : modules)
            module.update();

        if (DriverStation.isDisabled())
            stop();

        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getPositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                            modulePositions[moduleIndex].distanceMeters
                                    - lastModulePositions[moduleIndex].distanceMeters,
                            modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }


        // Update gyro angle
        if (gyro.isConnected()) {
            // Use the real gyro angle
            rawGyroRotation = yawPosition;
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply odometry update
        poseEstimator.update(rawGyroRotation, modulePositions);
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() { return kinematics.toChassisSpeeds(getStates()); }

    /** @return The current {@link Rotation2d} heading of the {@link Robot}. */
    public Rotation2d getHeading() { return yawPosition; }

    public void setStates(SwerveModuleState[] states) {
        if (states.length != modules.length)
            return;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_MPS);
        for (int i=0; i<modules.length; i++) {
            modules[i].setState(states[i]);
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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        setSpeeds(speeds);
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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        setSpeeds(speeds);
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, MAX_SPEED_MPS);

        for (int i = 0; i < modules.length; i++) {
            modules[i].setState(states[i]);
        }
    }

    public void lock() {
        setStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
    }

    public void reset(Pose2d pose) {
        gyro.reset();
        poseEstimator.resetPosition(yawPosition, getPositions(), pose);
    }

    public void reset() { reset(new Pose2d()); }
    public SwerveModule[] getModules() { return this.modules; }
    public SwerveDriveKinematics getKinematics() { return kinematics; }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i=0; i<modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i=0; i<modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    public Pose2d getPose() { return poseEstimator.getEstimatedPosition(); }
    public Rotation2d getRotation() { return getPose().getRotation(); }
}
