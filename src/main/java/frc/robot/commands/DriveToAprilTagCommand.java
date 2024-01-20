package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.math.ExtendedMath;

import java.util.Optional;

import static frc.robot.Constants.Chassis.MAX_SPEED_MPS;

public class DriveToAprilTagCommand extends Command {
    private final Pose2d desiredPose;
    private final int id;
    private final double targetHeightMeters;

    private Pose2d currentPose;
    private boolean noTarget, firstTarget;
    private long initTimeout = System.currentTimeMillis() + 5000;

    public DriveToAprilTagCommand(Pose2d desiredPose, double targetHeightMeters, int id) {
        this.currentPose = new Pose2d();
        this.desiredPose = desiredPose;
        this.targetHeightMeters = targetHeightMeters;
        this.noTarget = false;
        this.firstTarget = false;
        this.id = id;
        addRequirements(Robot.swerve);
    }

    public DriveToAprilTagCommand(Pose2d desiredPose, double targetHeightMeters) {
        this(desiredPose, 0, 0);
    }

    @Override
    public void initialize() {
        Robot.camera.setTargetHeight(targetHeightMeters);

        initTimeout = System.currentTimeMillis() + 5000;
        noTarget = false;
        firstTarget = true;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        Optional<Pose2d> storedPose = Robot.camera.getTrackedPose();
        if (storedPose.isEmpty() || (id > 0 && Robot.camera.getAprilTagID() != id)) {
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

        PIDController driveController = Robot.camera.getController();
        double jX = MathUtil.clamp(driveController.calculate(currentPose.getX(), desiredPose.getX()), -1, 1);
        double jY = MathUtil.clamp(driveController.calculate(currentPose.getY(), desiredPose.getY()),-1,1);

        ChassisSpeeds speeds = new ChassisSpeeds(
                jX * MAX_SPEED_MPS,
                jY * MAX_SPEED_MPS,
                0
        );
        Robot.swerve.driveRobotRelative(speeds, false);
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
                    }, false);
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
        return noTarget ||
                (
                        ExtendedMath.inTolerance(desiredPose.getX(), currentPose.getX(), 0.1)
                        && ExtendedMath.inTolerance(desiredPose.getY(), currentPose.getY(), 0.1)
                );
    }
}
