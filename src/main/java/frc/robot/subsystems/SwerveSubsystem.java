// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.File;
import java.io.IOException;
import java.util.List;
import java.util.Map;

import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveHIDBase;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import static frc.robot.Constants.AlertConfig.STRING_SWERVE_CONFIG_FAIL;
import static frc.robot.Constants.DriveConfig.MAX_SPEED_MPS;
import static frc.robot.Constants.DriveConfig.SWERVE_TELEMETRY;

public class SwerveSubsystem extends SwerveDrive implements Subsystem {

    private SwerveDrive swerveDrive;
    private SwerveAutoBuilder autoBuilder = null;
    public boolean fieldOriented = true;
    public boolean closedLoop = true;

    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

    /** @return A {@link Command} used to toggle teleoperated closed-loop. */
    public Command toggleClosedLoopCommand() { return Commands.runOnce(() -> closedLoop = !closedLoop); }

    /**
     * Initialize {@link SwerveDrive} with the directory provided.
     *
     * @param directory Directory of swerve drive config files.
     */
    public SwerveSubsystem(File directory) {
        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary objects being created.
        SwerveDriveTelemetry.verbosity = SWERVE_TELEMETRY;

        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(MAX_SPEED_MPS);

            // Heading correction should only be used while controlling the robot via angle.
            swerveDrive.setHeadingCorrection(false);
        } catch (Exception e) {
            IOManager
                    .getAlert(STRING_SWERVE_CONFIG_FAIL, AlertType.ERROR)
                    .setPersistence(true)
                    .setEnabled(true);
        }
    }

    /**
     * The primary method for controlling the drivebase.  Takes a {@link Translation2d} and a rotation rate, and
     * calculates and commands module states accordingly.  Can use either open-loop or closed-loop velocity control for
     * the wheel velocities.  Also has field- and robot-relative modes, which affect how the translation vector is
     * used.
     *
     * @param translation   {@link Translation2d} that is the commanded linear velocity of the robot, in meters per
     *                      second. In robot-relative mode, positive x is torwards the bow (front) and positive y is
     *                      torwards port (left).  In field-relative mode, positive x is away from the alliance wall
     *                      (field North) and positive y is torwards the left wall when looking through the driver
     *                      station glass (field West).
     * @param rotation      Robot angular rate, in radians per second. CCW positive.  Unaffected by field/robot
     *                      relativity.
     * @param fieldRelative Drive mode.  True for field-relative, false for robot-relative.
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
        swerveDrive.drive(translation,
                rotation,
                fieldRelative,
                false); // Open loop is disabled since it shouldn't be used most of the time.
    }

    /**
     * Drives the Robot using one Joystick.
     * @param stick The {@link DriveHIDBase} to use.
     */
    public void drive(DriveHIDBase stick, boolean fieldRelative, boolean isOpenLoop) {
        // Calculate the maximum speed based on XY and Twist.
        double xS = stick.getRobotX() * MAX_SPEED_MPS;
        double yS = stick.getRobotY() * MAX_SPEED_MPS;
        double tS = stick.getRobotTwist() * swerveDrive.swerveController.config.maxAngularVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldRelative)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, swerveDrive.getYaw());

        swerveDrive.drive(speeds, isOpenLoop, new Translation2d());
    }

    /**
     * Drives the Robot using two Joysticks: one for XY and one for Twist.
     * @param xyStick The {@link DriveHIDBase} to use for directional control.
     * @param twistStick The {@link DriveHIDBase} to use for twisting.
     */
    public void drive(DriveHIDBase xyStick, DriveHIDBase twistStick, boolean fieldRelative, boolean isOpenLoop) {
        // Calculate the maximum speed based on XY and Twist.
        double xS = xyStick.getRobotX() * MAX_SPEED_MPS;
        double yS = xyStick.getRobotY() * MAX_SPEED_MPS;
        double tS = twistStick.getRobotTwist() * swerveDrive.swerveController.config.maxAngularVelocity;

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldRelative)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, swerveDrive.getYaw());

        swerveDrive.drive(speeds, isOpenLoop, new Translation2d());
    }

    /**
     * Drive the robot given a chassis field oriented velocity.
     *
     * @param velocity Velocity according to the field.
     */
    public void driveFieldOriented(ChassisSpeeds velocity) { swerveDrive.driveFieldOriented(velocity); }

    /**
     * Drive according to the chassis robot oriented velocity.
     *
     * @param velocity Robot oriented {@link ChassisSpeeds}
     */
    public void drive(ChassisSpeeds velocity) { swerveDrive.drive(velocity); }

    /**
     * Get the swerve drive kinematics object.
     *
     * @return {@link SwerveDriveKinematics} of the swerve drive.
     */
    public SwerveDriveKinematics getKinematics() { return swerveDrive.kinematics; }

    /**
     * Resets odometry to the given pose. Gyro angle and module positions do not need to be reset when calling this
     * method.  However, if either gyro angle or module position is reset, this must be called in order for odometry to
     * keep working.
     *
     * @param initialPose The pose to set the odometry to
     */
    public void resetOdometry(Pose2d initialPose) { swerveDrive.resetOdometry(initialPose); }

    /**
     * Gets the current pose (position and rotation) of the robot, as reported by odometry.
     *
     * @return The robot's pose
     */
    public Pose2d getPose() { return swerveDrive.getPose(); }

    /**
     * Set chassis speeds with closed-loop velocity control.
     *
     * @param chassisSpeeds Chassis Speeds to set.
     */
    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) { swerveDrive.setChassisSpeeds(chassisSpeeds); }

    /**
     * Post the trajectory to the field.
     *
     * @param trajectory The trajectory to post.
     */
    public void postTrajectory(Trajectory trajectory) { swerveDrive.postTrajectory(trajectory); }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but facing toward 0.
     */
    public void zeroGyro() { swerveDrive.zeroGyro(); }

    /**
     * Sets the drive motors to brake/coast mode.
     *
     * @param brake True to set motors to brake mode, false for coast.
     */
    public void setMotorBrake(boolean brake) { swerveDrive.setMotorIdleMode(brake); }

    /**
     * Gets the current yaw angle of the robot, as reported by the imu.  CCW positive, not wrapped.
     *
     * @return The yaw angle
     */
    public Rotation2d getHeading() { return swerveDrive.getYaw(); }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() { return swerveDrive.getFieldVelocity(); }

    /**
     * Gets the current velocity (x, y and omega) of the robot
     *
     * @return A {@link ChassisSpeeds} object of the current velocity
     */
    public ChassisSpeeds getRobotVelocity() { return swerveDrive.getRobotVelocity(); }

    /**
     * Get the {@link SwerveController} in the swerve drive.
     *
     * @return {@link SwerveController} from the {@link SwerveDrive}.
     */
    public SwerveController getSwerveController() { return swerveDrive.swerveController; }

    /**
     * Get the {@link SwerveDriveConfiguration} object.
     *
     * @return The {@link SwerveDriveConfiguration} fpr the current drive.
     */
    public SwerveDriveConfiguration getSwerveDriveConfiguration() { return swerveDrive.swerveDriveConfiguration; }

    /**
     * Lock the swerve drive to prevent it from moving.
     */
    public void lock() { swerveDrive.lockPose(); }

    /**
     * Gets the current pitch angle of the robot, as reported by the imu.
     *
     * @return The heading as a {@link Rotation2d} angle
     */
    public Rotation2d getPitch() { return swerveDrive.getPitch(); }

    /**
     * Factory to fetch the PathPlanner command to follow the defined path.
     *
     * @param path             Path planner path to specify.
     * @param constraints      {@link PathConstraints} for {@link com.pathplanner.lib.PathPlanner#loadPathGroup}
     *                         function limiting velocity and acceleration.
     * @param eventMap         {@link java.util.HashMap} of commands corresponding to path planner events given as
     *                         strings.
     * @param translation      The {@link PIDConstants} for the translation of the robot while following the path.
     * @param rotation         The {@link PIDConstants} for the rotation of the robot while following the path.
     * @param useAllianceColor Automatically transform the path based on alliance color.
     * @return PathPlanner command to follow the given path.
     */
    public Command creatPathPlannerCommand(String path, PathConstraints constraints, Map<String, Command> eventMap,
                                           PIDConstants translation, PIDConstants rotation, boolean useAllianceColor) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(path, constraints);
        if (autoBuilder == null) {
            autoBuilder = new SwerveAutoBuilder(
                    swerveDrive::getPose,
                    swerveDrive::resetOdometry,
                    translation,
                    rotation,
                    swerveDrive::setChassisSpeeds,
                    eventMap,
                    useAllianceColor,
                    this
            );
        }

        return autoBuilder.fullAuto(pathGroup);
    }
}
