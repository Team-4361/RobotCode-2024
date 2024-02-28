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
    private Pose2d currentPose = new Pose2d();

    public DriveToPoseCommand(Pose2d desiredPose) {
        addRequirements(Robot.swerve);
        this.desiredPose = desiredPose;
    }

    private ChassisSpeeds calculateSpeeds() {
        PIDController driveController = Robot.shooterCamera.getDriveController();
        PIDController turnController = Robot.shooterCamera.getTurnController();

        double mX = Robot.shooterCamera.getMaxDriveSpeed();
        double jX = MathUtil.clamp(driveController.calculate(currentPose.getX(), desiredPose.getX()), -mX, mX);
        double jY = MathUtil.clamp(driveController.calculate(currentPose.getY(), desiredPose.getY()), -mX, mX);
        double jO = MathUtil.clamp(
                turnController.calculate(
                        currentPose.getRotation().getRadians(),
                        desiredPose.getRotation().getRadians()
                ),
                -PHOTON_TURN_MAX_SPEED,
                PHOTON_TURN_MAX_SPEED
        );
        return new ChassisSpeeds(
                jX * MAX_SPEED_MPS,
                jY * MAX_SPEED_MPS,
                jO * MAX_SPEED_MPS
        );
    }

    @Override
    public void execute() {
        currentPose = Robot.swerve.getPose();
        Robot.swerve.setChassisSpeeds(calculateSpeeds());
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
        return GlobalUtils.inTolerance(desiredPose.getX(), currentPose.getX(), 0.1)
                && GlobalUtils.inTolerance(desiredPose.getY(), currentPose.getY(), 0.1);
    }
}
