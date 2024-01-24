package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.swerve.SwerveModule;

import java.util.Optional;

import static frc.robot.Constants.AlertConfig.STRING_GYRO_CALIBRATING;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.LooperConfig.*;

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
    private final SwerveDriveOdometry odometry;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;

    // Used for keeping the gyro angle at zero when the floor
    // is not PERFECTLY flat. A YAW OFFSET is not required due to integrated
    // functionality in the gyroscope.
    private double rollOffset;
    private double pitchOffset;

    public static boolean fieldOriented = true;
    public static boolean closedLoop = false;

    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

    /** @return A {@link Command} used to toggle teleoperated closed-loop. */
    public Command toggleClosedLoopCommand() { return Commands.runOnce(() -> closedLoop = !closedLoop); }

    public Command resetCommand() { return Commands.runOnce(this::reset); }

    /** Stops the {@link SwerveDriveSubsystem} from moving. */
    public void stop() {
        setStates(new SwerveModuleState[]
                {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0))
                }, false);
    }

    /**
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules.
     * @param frontLeft  The front left {@link SwerveModule} to use.
     * @param frontRight The front right {@link SwerveModule} to use.
     * @param backLeft   The back left {@link SwerveModule} to use.
     * @param backRight  The back right {@link SwerveModule} to use.
     */
    public SwerveDriveSubsystem(SwerveModule frontLeft,
                                SwerveModule frontRight,
                                SwerveModule backLeft,
                                SwerveModule backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.pitchOffset = 0;
        this.rollOffset = 0;

        this.gyro = new AHRS(SPI.Port.kMXP);
        this.gyro.reset();

        // Define the distances relative to the center of the robot. +X is forward and +Y is left.
        double SWERVE_CHASSIS_SIDE_LENGTH = CHASSIS_MODE.getSideLength();
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2),  // FL
                new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2), // FR
                new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2), // BL
                new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2) // BR
        );
        this.odometry = new SwerveDriveOdometry(kinematics, getHeading(), getPositions());
        IOManager.getAlert(STRING_GYRO_CALIBRATING, AlertType.WARNING)
                .setCondition(gyro::isCalibrating)
                .setPersistent(false)
                .setDisableDelay(2000)
                .setOneUse(false);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::reset,
                this::getRobotVelocity,
                this::drive,
                new HolonomicPathFollowerConfig(
                        CHASSIS_MODE.getAutoDrivePID(),
                        CHASSIS_MODE.getAutoTurnPID(),
                        CHASSIS_MODE.getMaxSpeed(),
                        SWERVE_CHASSIS_SIDE_LENGTH/2,
                        new ReplanningConfig()
                ), () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );

        //FRCSparkMax.burnAllFlash();

        IOManager.addPeriodicIfExists(STRING_ODOMETRY_NAME, () -> odometry.update(getHeading(), getPositions()));
        IOManager.addPeriodicIfExists(STRING_DASHBOARD_NAME, () -> {
            SmartDashboard.putString("Odometry", getPose().toString());
            frontLeft.updateDashboard();
            frontRight.updateDashboard();
            backLeft.updateDashboard();
            backRight.updateDashboard();
        });
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
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return kinematics.toChassisSpeeds(getStates());
    }

    /** @return The current roll of the {@link Robot} in <b>degrees</b>. */
    public double getRoll() { return gyro.getRoll() - rollOffset; }

    /** @return The current pitch of the {@link Robot} in <b>degrees</b>. */
    public double getPitch() { return gyro.getPitch() - pitchOffset; }

    /** @return The current yaw of the {@link Robot} in <b>degrees</b> (0-360) */
    public double getYaw() { return gyro.getAngle() % 360; }

    public Rotation2d getHeading() { return gyro.getRotation2d(); }

    public void setStates(SwerveModuleState[] states, boolean isClosedLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, CHASSIS_MODE.getMaxSpeed());

        frontLeft.setState(states[0], isClosedLoop);
        frontRight.setState(states[1], isClosedLoop);
        backLeft.setState(states[2], isClosedLoop);
        backRight.setState(states[3], isClosedLoop);
    }

    /** The registered {@link SwerveModuleState} list. */
    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{
                frontLeft.getState(),
                frontRight.getState(),
                backLeft.getState(),
                backRight.getState()
        };
    }

    /**
     * Drives the Robot using one Joystick.
     * @param stick The {@link DriveHIDBase} to use.
     */
    public void drive(DriveHIDBase stick) {
        // Calculate the maximum speed based on XY and Twist.
        double xS = stick.getRobotX() * CHASSIS_MODE.getMaxSpeed();
        double yS = stick.getRobotY() * CHASSIS_MODE.getMaxSpeed();
        double tS = stick.getRobotTwist() * CHASSIS_MODE.getMaxSpeed();

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        driveRobotRelative(speeds, closedLoop);
    }

    /**
     * Drives the Robot using two Joysticks (one for XY, one for Twist)
     * @param xyStick The {@link DriveHIDBase} to use for XY translation.
     * @param twistStick The {@link DriveHIDBase} to use for Twist.
     */
    public void drive(DriveHIDBase xyStick, DriveHIDBase twistStick) {
        double xS = xyStick.getRobotX() * CHASSIS_MODE.getMaxSpeed();
        double yS = xyStick.getRobotY() * CHASSIS_MODE.getMaxSpeed();
        double tS = twistStick.getRobotTwist() * CHASSIS_MODE.getMaxSpeed();

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        driveRobotRelative(speeds, closedLoop);
    }

    /**
     * Drives the Robot without field orientated using an input {@link ChassisSpeeds}.
     * @param speeds The {@link ChassisSpeeds} to use.
     */
    public void drive(ChassisSpeeds speeds) {setStates(kinematics.toSwerveModuleStates(speeds), false); }

    public void driveRobotRelative(ChassisSpeeds speeds, boolean isClosedLoop) {
        setStates(kinematics.toSwerveModuleStates(speeds), isClosedLoop);
    }

    public void lock() {
        setStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90))
        }, false);
    }

    public void reset(Pose2d pose) {
        rollOffset = gyro.getRoll();
        pitchOffset = gyro.getPitch();
        gyro.reset();
        odometry.resetPosition(gyro.getRotation2d(), getPositions(), pose);
    }

    public void reset() { reset(new Pose2d()); }

    public SwerveModule getFrontLeft() { return frontLeft; }
    public SwerveModule getFrontRight() { return frontRight; }
    public SwerveModule getBackLeft() { return backLeft; }
    public SwerveModule getBackRight() { return backRight; }
    public SwerveDriveKinematics getKinematics() { return kinematics; }
    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[]{
                frontLeft.getPosition(),
                frontRight.getPosition(),
                backLeft.getPosition(),
                backRight.getPosition()
        };
    }
    public Pose2d getPose() { return odometry.getPoseMeters(); }
}
