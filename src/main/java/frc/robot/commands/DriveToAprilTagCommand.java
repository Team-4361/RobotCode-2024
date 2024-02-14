package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.math.GlobalUtils;

import java.util.Optional;

public class DriveToAprilTagCommand extends Command {
    private final Pose2d desiredPose;
    private final int id;
    private final double targetHeightMeters;

    private Pose2d currentPose;
    private boolean noTarget, firstTarget, stopOnEnd;
    private long initTimeout = System.currentTimeMillis() + 5000;

    public DriveToAprilTagCommand(Pose2d desiredPose, double targetHeightMeters, int id, boolean stopOnEnd) {
        addRequirements(Robot.swerve);
        this.desiredPose = desiredPose;
        this.targetHeightMeters = targetHeightMeters;
        this.noTarget = false;
        this.firstTarget = false;
        this.id = id;
        addRequirements(Robot.swerve);
    }

    public DriveToAprilTagCommand(Pose2d desiredPose, double targetHeightMeters, boolean stopOnEnd) {
        this(desiredPose, targetHeightMeters, 0, stopOnEnd);
    }

    @Override
    public void initialize() {
        Robot.frontCamera.setTargetHeight(targetHeightMeters);

        initTimeout = System.currentTimeMillis() + 5000;
        noTarget = false;
        firstTarget = true;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        Optional<Pose2d> storedPose = Robot.frontCamera.getTrackedPose();
        if (storedPose.isEmpty() || (id > 0 && Robot.frontCamera.getAprilTagID() != id)) {
            Robot.swerve.stop();
            if (!firstTarget && System.currentTimeMillis() > initTimeout) {
                noTarget = true;
                currentPose = new Pose2d();
            }
            return;
        }

        if (firstTarget)
            firstTarget = false;
        currentPose = storedPose.get();

        Robot.swerve.driveRobotRelative(calculateSpeeds());
    }

    private ChassisSpeeds calculateSpeeds() {
        PIDController driveController = Robot.frontCamera.getDriveController();
        PIDController turnController = Robot.frontCamera.getTurnController();

        double mX = PHOTON_DRIVE_MAX_SPEED.getValue();
        double jX = MathUtil.clamp(driveController.calculate(currentPose.getX(), desiredPose.getX()), -mX, mX);
        double jY = MathUtil.clamp(driveController.calculate(currentPose.getY(), desiredPose.getY()),-mX,mX);
        double jO = MathUtil.clamp(
                turnController.calculate(
                    currentPose.getRotation().getRadians(),
                    desiredPose.getRotation().getRadians()
                ),
                -PHOTON_TURN_MAX_SPEED.getValue(),
                PHOTON_TURN_MAX_SPEED.getValue()
        );
        return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(
                jX * CHASSIS_MODE.getMaxSpeed(),
                jY * CHASSIS_MODE.getMaxSpeed(),
                jO * CHASSIS_MODE.getMaxSpeed()
        ), Robot.swerve.getHeading());
    }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * <p>Do not schedule commands here that share requirements with this command. Use {@link
     * #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override
    public void end(boolean interrupted) {
        if (DriverStation.isAutonomous()) {
            Robot.swerve.lock();
        } else {
            Robot.swerve.setStates(new SwerveModuleState[]
                    {
                            new SwerveModuleState(0, new Rotation2d(0)),
                            new SwerveModuleState(0, new Rotation2d(0)),
                            new SwerveModuleState(0, new Rotation2d(0)),
                            new SwerveModuleState(0, new Rotation2d(0))
                    });
        }
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        if (stopOnEnd) {
            return noTarget ||
                    (
                            GlobalUtils.inTolerance(desiredPose.getX(), currentPose.getX(), 0.1)
                                    && GlobalUtils.inTolerance(desiredPose.getY(), currentPose.getY(), 0.1)
                    );
        } else {
            return false;
        }
    }
}
