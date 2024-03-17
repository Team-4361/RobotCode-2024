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
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.DashTunablePID;
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
public class SwerveDriveSubsystem extends SubsystemBase {
    private final Alert focDisabledAlert;
    private final PIDController autoDriveController;
    private final PIDController autoTurnController;
    private final DashTunablePID autoDriveTune;
    private final DashTunablePID autoTurnTune;
    private final DashTunableNumber autoSpeedTune;
    private final SwerveDrive swerveDrive;

    public boolean fieldOriented = true;
    public boolean slowMode = false;

    public boolean hasResetGyro = false;

    private double maxAutoDriveSpeed = AUTO_DRIVE_MAX_SPEED;

    public PIDController getAutoDriveController() { return this.autoDriveController; }
    public PIDController getAutoTurnController()  { return this.autoTurnController;  }

    public void setMaxAutoDriveSpeed(double speed) { this.maxAutoDriveSpeed = speed; }
    public double getMaxAutoDriveSpeed() { return maxAutoDriveSpeed; }

    public Pose2d getPose() { return swerveDrive.getPose(); }
    public ChassisSpeeds getRobotVelocity() { return swerveDrive.getRobotVelocity(); }
    public void setChassisSpeeds(ChassisSpeeds speeds) { swerveDrive.setChassisSpeeds(speeds); }

    public void lockPose() { swerveDrive.lockPose(); }

    /**
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules.
     */
    public SwerveDriveSubsystem() {
        try {
            swerveDrive = new SwerveParser(new File(getDeployDirectory(), "swerve"))
                    .createSwerveDrive(MAX_SPEED_MPS);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        this.autoDriveController = GlobalUtils.generateController(AUTO_DRIVE_PID);
        this.autoTurnController = GlobalUtils.generateController(AUTO_TURN_PID);
        this.focDisabledAlert = new Alert("Swerve FOC disabled!", Alert.AlertType.WARNING);

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setMotorIdleMode(true);
        SwerveDriveTelemetry.verbosity = SWERVE_TUNING_ENABLED ? HIGH : MACHINE;

        if (SWERVE_TUNING_ENABLED) {
            this.autoDriveTune = new DashTunablePID("Swerve: Auto Drive PID", AUTO_DRIVE_PID);
            this.autoTurnTune = new DashTunablePID("Swerve: Auto Turn PID", AUTO_TURN_PID);
            this.autoSpeedTune = new DashTunableNumber("Swerve: AD Speed", AUTO_DRIVE_MAX_SPEED);

            autoSpeedTune.addConsumer(this::setMaxAutoDriveSpeed);
            autoDriveTune.addConsumer(autoDriveController::setP, autoDriveController::setI, autoDriveController::setD);
            autoTurnTune.addConsumer(autoTurnController::setP, autoTurnController::setI, autoTurnController::setD);
        } else {
            autoDriveTune = null;
            autoTurnTune = null;
            autoSpeedTune = null;
        }
    }

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

    public ChassisSpeeds calculateSpeedsToPose(Pose2d currentPose, Pose2d desiredPose, boolean usePhoton) {
        PIDController driveController, turnController;
        double mX, mO;

        if (usePhoton) {
            driveController = Robot.shooterCamera.getDriveController();
            turnController = Robot.shooterCamera.getTurnController();
            mX = Robot.shooterCamera.getMaxDriveSpeed();
            mO = PHOTON_TURN_MAX_SPEED;
        } else {
            driveController = Robot.swerve.getAutoDriveController();
            turnController = Robot.swerve.getAutoTurnController();
            mX = Robot.swerve.getMaxAutoDriveSpeed();
            mO = AUTO_TURN_MAX_SPEED;
        }

        double jX = MathUtil.clamp(driveController.calculate(currentPose.getX(), desiredPose.getX()), -mX, mX);
        double jY = MathUtil.clamp(driveController.calculate(currentPose.getY(), desiredPose.getY()), -mX, mX);
        double jO = MathUtil.clamp(
                turnController.calculate(
                        currentPose.getRotation().getRadians(),
                        desiredPose.getRotation().getRadians()
                ),
                -mO,
                mO
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

    public void setStates(SwerveModuleState[] states) { swerveDrive.setModuleStates(states, false); }

    public void addVisionMeasurement(Pose2d pose, double timestamp) {
        swerveDrive.addVisionMeasurement(pose, timestamp);
    }

    @Override
    public void periodic() {
        if (SWERVE_TUNING_ENABLED) {
            SmartDashboard.putNumber("FL Turn", swerveDrive.getModuleMap().get("frontleft").getAbsolutePosition());
            SmartDashboard.putNumber("FR Turn", swerveDrive.getModuleMap().get("frontright").getAbsolutePosition());
            SmartDashboard.putNumber("BL Turn", swerveDrive.getModuleMap().get("backleft").getAbsolutePosition());
            SmartDashboard.putNumber("BR Turn", swerveDrive.getModuleMap().get("backright").getAbsolutePosition());
        }
        if (autoTurnTune != null)
            autoTurnTune.update();
        if (autoDriveTune != null)
            autoDriveTune.update();
        if (autoSpeedTune != null)
            autoSpeedTune.update();

        SmartDashboard.putString("Pose", getPose().toString());
        focDisabledAlert.set(!fieldOriented);
    }

    public void reset(Pose2d pose) {
        swerveDrive.zeroGyro();
        swerveDrive.resetOdometry(pose);
        swerveDrive.synchronizeModuleEncoders();
        if (DriverStation.isTeleop())
            hasResetGyro = true;
    }

    public void rawDrive(
            Translation2d translation,
            double rotation,
            boolean fieldRelative,
            boolean isOpenLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, isOpenLoop);
    }

    public double getMaximumVelocity() { return swerveDrive.getMaximumVelocity(); }
    public double getMaximumAngularVelocity() { return swerveDrive.getMaximumAngularVelocity(); }

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
            swerveDrive.drive(
                new Translation2d(
                        Math.pow(translationX.getAsDouble(), 3) * swerveDrive.getMaximumVelocity(),
                        Math.pow(translationY.getAsDouble(), 3) * swerveDrive.getMaximumVelocity()
                ),
                Math.pow(angularRotationX.getAsDouble(), 3) * swerveDrive.getMaximumAngularVelocity(),
                fieldOriented,
                false
            );
        });
    }

    public void reset() { reset(new Pose2d()); }
}
