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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.pid.DashTunablePID;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.math.SwerveMath;
import swervelib.parser.PIDFConfig;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.Optional;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.SWERVE_TUNING_ENABLED;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveModule}s, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    public boolean fieldOriented = true;
    private final SwerveDrive drive;
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private long nextCheck = System.currentTimeMillis();

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

    public void updateDrivePID(PIDFConfig configs) {
        SwerveModule[] modules = drive.getModules();
        for (SwerveModule module : modules) {
            module.configuration.velocityPIDF = configs;
        }
    }

    public void updateTurnPID(PIDFConfig configs) {
        SwerveModule[] modules = drive.getModules();

        for (SwerveModule module : modules) {
            module.configuration.anglePIDF = configs;
        }
    }

    /**
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules.
     */
    public SwerveDriveSubsystem() {
        try {
            drive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
                    .createSwerveDrive(MAX_SPEED_MPS);
            /*
                    SwerveMath.calculateDegreesPerSteeringRotation(12.8),
                    SwerveMath.calculateMetersPerRotation(Units.inchesToMeters(4), 6.75)
            );

             */
        } catch (IOException e) {
            throw new RuntimeException(e);
        }

        drive.setHeadingCorrection(false);
        drive.setCosineCompensator(!SwerveDriveTelemetry.isSimulation);

        if (SWERVE_TUNING_ENABLED) {
            driveTune = new DashTunablePID("Swerve: Drive PID", DRIVE_PID);
            turnTune = new DashTunablePID("Swerve: Turn PID", TURN_PID);
        } else {
            driveTune = null;
            turnTune = null;
        }

        // Update the PID values
        updateDrivePID(DRIVE_PID.toPIDF());
        updateTurnPID(TURN_PID.toPIDF());

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
        return drive.getFieldVelocity();
    }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        drive.updateOdometry();

        SwerveModule flModule = drive.getModuleMap().get("frontleft");
        SwerveModule frModule = drive.getModuleMap().get("frontright");
        SwerveModule blModule = drive.getModuleMap().get("backleft");
        SwerveModule brModule = drive.getModuleMap().get("backright");

        SmartDashboard.putNumber("FL Pos", flModule.getAbsolutePosition());
        SmartDashboard.putNumber("FR Pos", frModule.getAbsolutePosition());
        SmartDashboard.putNumber("BL Pos", blModule.getAbsolutePosition());
        SmartDashboard.putNumber("BR Pos", brModule.getAbsolutePosition());

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
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        drive.setChassisSpeeds(speeds);
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() { return drive.getRobotVelocity(); }

    /** @return The current {@link Rotation2d} heading of the {@link Robot}. */
    public Rotation2d getHeading() { return drive.getOdometryHeading(); }

    public void setStates(SwerveModuleState[] states) {
        drive.setModuleStates(states, false);
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

        drive.drive(speeds);
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

        drive.drive(speeds);
    }

    public void lock() {
        drive.lockPose();
    }

    public void reset(Pose2d pose) {
        drive.zeroGyro();
        drive.resetDriveEncoders();
        drive.resetOdometry(pose);
    }

    public void reset() { reset(new Pose2d()); }
    public SwerveDriveKinematics getKinematics() { return drive.kinematics; }

    public SwerveModulePosition[] getPositions() {
        return drive.getModulePositions();
    }

    public SwerveModuleState[] getStates() {
        return drive.getStates();
    }

    public Pose2d getPose() { return drive.getPose(); }
    public Rotation2d getRotation() { return getPose().getRotation(); }
}
