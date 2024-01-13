package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.swerve.SwerveModule;

import static frc.robot.Constants.AlertConfig.STRING_GYRO_CALIBRATING;
import static frc.robot.Constants.Chassis.MAX_SPEED_MPS;
import static frc.robot.Constants.Chassis.SWERVE_CHASSIS_SIDE_LENGTH;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveChassis}, and utilizing
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

    public SwerveDriveSubsystem(SwerveModule frontLeft,
                                SwerveModule frontRight,
                                SwerveModule backLeft,
                                SwerveModule backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.pitchOffset = 0;
        this.rollOffset = 0;

        // Define the distances relative to the center of the robot. +X is forward and +Y is left.
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2),  // FL
                new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2), // FR
                new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2), // BL
                new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2) // BR
        );
        this.odometry = new SwerveDriveOdometry(kinematics, getRotation(), getPositions());

        gyro.reset();
        IOManager.getAlert(STRING_GYRO_CALIBRATING, AlertType.WARNING)
                .setCondition(gyro::isCalibrating)
                .setPersistence(false)
                .setDisableDelay(1000)
                .setOneUse(false);
    }


    public double getRoll() { return gyro.getRoll() - rollOffset; }
    public double getPitch() { return gyro.getPitch() - pitchOffset; }

    /** @return The yaw of the Robot in degrees from (0-360 degrees) */
    public double getYaw() { return gyro.getAngle() % 360; }

    public Rotation2d getRotation() { return gyro.getRotation2d(); }

    public void setStates(SwerveModuleState[] states, boolean isClosedLoop) {
        frontLeft.setState(states[0], isClosedLoop);
        frontRight.setState(states[1], isClosedLoop);
        backLeft.setState(states[2], isClosedLoop);
        backRight.setState(states[3], isClosedLoop);
    }

    @Override
    public void periodic() {
        odometry.update(getRotation(), getPositions());
        frontLeft.updateDashboard();
        frontRight.updateDashboard();
        backLeft.updateDashboard();
        backRight.updateDashboard();
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
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());

        drive(speeds, closedLoop);
    }

    public void drive(DriveHIDBase xyStick, DriveHIDBase twistStick) {
        double xS = xyStick.getRobotX() * MAX_SPEED_MPS;
        double yS = xyStick.getRobotY() * MAX_SPEED_MPS;
        double tS = twistStick.getRobotTwist() * MAX_SPEED_MPS;

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getRotation());

        drive(speeds, closedLoop);
    }

    public void drive(ChassisSpeeds speeds, boolean isClosedLoop) {
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
