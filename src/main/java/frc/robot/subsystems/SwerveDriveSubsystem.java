package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.base.BaseSubsystem;
import swervelib.SwerveDrive;
import swervelib.SwerveModule;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.Alert;
import swervelib.telemetry.SwerveDriveTelemetry;

import java.io.File;
import java.io.IOException;
import java.util.HashMap;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static edu.wpi.first.wpilibj.Filesystem.getDeployDirectory;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Systems.SWERVE;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.HIGH;
import static swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity.MACHINE;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveModule}s, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem extends BaseSubsystem {
    private final Alert focDisabledAlert;
    private final Supplier<PIDController> autoDrivePID;
    private final Supplier<PIDController> autoTurnPID;
    //private final TunablePID autoDriveTune;
    //private final TunablePID autoTurnTune;
    //private final TunableNumber autoSpeedTune;
    private final SwerveDrive swerveDrive;

    public boolean fieldOriented = true;
    public boolean slowMode = false;

    public boolean hasResetGyro = false;

    private double maxAutoDriveSpeed = AUTO_DRIVE_MAX_SPEED;

    public PIDController getAutoDrivePID() { return autoDrivePID.get(); }
    public PIDController getAutoTurnPID() { return autoTurnPID.get(); }

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
        super(SWERVE, new HashMap<>());

        try {
            swerveDrive = new SwerveParser(new File(getDeployDirectory(), "swerve"))
                    .createSwerveDrive(MAX_SPEED_MPS);
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        this.autoDrivePID = registerPID("AutoDrive", AUTO_DRIVE_PID);
        this.autoTurnPID = registerPID("AutoTurn", AUTO_TURN_PID);
        this.focDisabledAlert = new Alert("Swerve FOC disabled!", Alert.AlertType.WARNING);

        registerConstant("ADSpeed", AUTO_DRIVE_MAX_SPEED);

        swerveDrive.setHeadingCorrection(false);
        swerveDrive.setCosineCompensator(false);
        swerveDrive.setMotorIdleMode(true);
        SwerveDriveTelemetry.verbosity = isTuningEnabled() ? HIGH : MACHINE;

        /*
        if (SWERVE_TUNING_ENABLED) {
            this.autoDriveTune = new TunablePID("Swerve: Auto Drive PID", AUTO_DRIVE_PID);
            this.autoTurnTune = new TunablePID("Swerve: Auto Turn PID", AUTO_TURN_PID);
            this.autoSpeedTune = new TunableNumber("Swerve: AD Speed", AUTO_DRIVE_MAX_SPEED);

            autoSpeedTune.addConsumer(this::setMaxAutoDriveSpeed);
            autoDriveTune.addConsumer(autoDrivePID::setP, autoDrivePID::setI, autoDrivePID::setD);
            autoTurnTune.addConsumer(autoTurnPID::setP, autoTurnPID::setI, autoTurnPID::setD);
        } else {
            autoDriveTune = null;
            autoTurnTune = null;
            autoSpeedTune = null;
        }
         */

        setDashUpdate(() -> {
            if (isTuningEnabled()) {
                SmartDashboard.putNumber("FL Turn", swerveDrive.getModuleMap().get("frontleft").getAbsolutePosition());
                SmartDashboard.putNumber("FR Turn", swerveDrive.getModuleMap().get("frontright").getAbsolutePosition());
                SmartDashboard.putNumber("BL Turn", swerveDrive.getModuleMap().get("backleft").getAbsolutePosition());
                SmartDashboard.putNumber("BR Turn", swerveDrive.getModuleMap().get("backright").getAbsolutePosition());
            }
            SmartDashboard.putString("Pose", getPose().toString());
            focDisabledAlert.set(!fieldOriented);
        });
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
            mX = Robot.shooterCamera.getMaxDrivePower();
            mO = PHOTON_TURN_MAX_SPEED;
        } else {
            driveController = Robot.swerve.getAutoDrivePID();
            turnController = Robot.swerve.getAutoTurnPID();
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
