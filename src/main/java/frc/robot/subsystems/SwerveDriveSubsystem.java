package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.swerve.SwerveChassis;
import frc.robot.util.swerve.SwerveModule;
import frc.robot.util.swerve.SwerveOdometry;

import static frc.robot.Constants.Chassis.MAX_SPEED_MPS;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveChassis}, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem {
    private final SwerveDriveKinematics kinematics;
    private final SwerveOdometry odometry;
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;
    private final AHRS gyro;
    private final Rotation3d gyroOffset;

    public static boolean fieldOriented = true;
    public static boolean closedLoop = true;

    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

    /** @return A {@link Command} used to toggle teleoperated closed-loop. */
    public Command toggleClosedLoopCommand() { return Commands.runOnce(() -> closedLoop = !closedLoop); }

    public SwerveDriveSubsystem(SwerveModule frontLeft,
                                SwerveModule frontRight,
                                SwerveModule backLeft,
                                SwerveModule backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
        this.gyro = new AHRS(SPI.Port.kMXP);
        this.gyroOffset = new Rotation3d(0, 0, 0);
    }

    public void setStates(SwerveModuleState[] states, boolean isClosedLoop) {
        frontLeft.setState(states[0], isClosedLoop);
        frontRight.setState(states[1], isClosedLoop);
        backLeft.setState(states[2], isClosedLoop);
        backRight.setState(states[3], isClosedLoop);
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
}
