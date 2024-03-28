package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.auto.PhotonCameraModule;

import java.util.Optional;

public class DriveTargetCommand extends Command {
    private final Transform2d targetDistance;
    private final PhotonCameraModule camera;
    private final boolean stopOnEnd;

    private Transform2d currentDistance;
    private boolean noTarget;
    private boolean firstTarget;

    private long initTimeout = System.currentTimeMillis() + 5000;

    public DriveTargetCommand(PhotonCameraModule module,
                              int pipeline,
                              Transform2d targetDistance,
                              boolean stopOnEnd) {
        addRequirements(Robot.swerve, module);
        this.camera = module;
        this.targetDistance = targetDistance;
        this.noTarget = false;
        this.firstTarget = false;
        this.stopOnEnd = stopOnEnd;
        module.setPipeline(pipeline);
    }

    @Override
    public void initialize() {
        initTimeout = System.currentTimeMillis() + 5000;
        noTarget = false;
        firstTarget = true;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        Optional<Transform2d> storedPose = Robot.shooterCamera.getTrackedDistance();

        if (storedPose.isEmpty()) {
            Robot.swerve.stop();
            if (!firstTarget && System.currentTimeMillis() > initTimeout) {
                noTarget = true;
                currentDistance = new Transform2d();
            }
            return;
        }

        if (firstTarget)
            firstTarget = false;

        currentDistance = storedPose.get();
        Robot.swerve.setChassisSpeeds(calculateSpeeds());
    }

    public boolean atTarget() {
        return MathUtil.isNear(currentDistance.getX(), targetDistance.getX(), 0.1) &&
                MathUtil.isNear(currentDistance.getY(), targetDistance.getY(), 0.1) &&
                MathUtil.isNear(
                        currentDistance
                                .getRotation()
                                .getRadians(),
                        targetDistance
                                .getRotation()
                                .getRadians(),
                        0.1);
    }

    private ChassisSpeeds calculateSpeeds() {
        PIDController driveController = camera.getDriveController();
        PIDController turnController = camera.getTurnController();

        double mXY = camera.getMaxDrivePower();
        double mO = camera.getMaxTurnPower();

        double jX = MathUtil.clamp(driveController.calculate(currentDistance.getX(), targetDistance.getX()), -mXY, mXY);
        double jY = MathUtil.clamp(driveController.calculate(currentDistance.getY(), targetDistance.getY()), -mXY, mXY);
        double jO = MathUtil.clamp(
                turnController.calculate(
                        currentDistance.getRotation().getDegrees(),
                        targetDistance.getRotation().getDegrees()%180
                ),
                -mO,
                mO
        );
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                jX * Robot.swerve.getMaximumVelocity(),
                jY * Robot.swerve.getMaximumVelocity(),
                jO * Robot.swerve.getMaximumAngularVelocity(),
                Robot.swerve.getPose().getRotation()
        );
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
            Robot.swerve.stop();
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
            return noTarget || atTarget();
        } else {
            return false;
        }
    }
}
