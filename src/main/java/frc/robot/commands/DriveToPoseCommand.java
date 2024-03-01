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

import static frc.robot.Constants.Chassis.MAX_SPEED_MPS;
import static frc.robot.Constants.Chassis.PHOTON_TURN_MAX_SPEED;

public class DriveToPoseCommand extends Command {
    private final Pose2d desiredPose;

    public DriveToPoseCommand(Pose2d desiredPose) {
        addRequirements(Robot.swerve);
        this.desiredPose = desiredPose;
    }

    @Override
    public void execute() {
        Robot.swerve.setChassisSpeeds(Robot.swerve.calculateSpeedsToPose(desiredPose, false));
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
            Robot.swerve.lockPose();
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

    @Override
    public boolean isFinished() {
        Pose2d currentPose = Robot.swerve.getPose();
        return GlobalUtils.inTolerance(desiredPose.getX(), currentPose.getX(), 0.1)
                && GlobalUtils.inTolerance(desiredPose.getY(), currentPose.getY(), 0.1);
    }
}
