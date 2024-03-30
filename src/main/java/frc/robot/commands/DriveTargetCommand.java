package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.util.auto.PhotonCameraModule;

import java.util.Optional;

public class DriveTargetCommand extends Command {
    private final PhotonCameraModule camera;
    private final boolean stopOnEnd;

    private boolean noTarget;
    private boolean firstTarget;
    private final int pipeline;
    private final Transform2d target;

    private long initTimeout = System.currentTimeMillis() + 5000;

    public DriveTargetCommand(PhotonCameraModule module,
                              int pipeline,
                              Transform2d target,
                              boolean stopOnEnd) {
        addRequirements(Robot.swerve, module);
        this.camera = module;
        this.noTarget = false;
        this.firstTarget = false;
        this.target = target;
        this.stopOnEnd = stopOnEnd;
        this.pipeline = pipeline;
    }

    @Override
    public void initialize() {
        camera.setPipeline(pipeline);
        camera.setTarget(target);
        initTimeout = System.currentTimeMillis() + 5000;
        noTarget = false;
        firstTarget = true;
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (!camera.hasTarget()) {
            Robot.swerve.stop();
            if (!firstTarget && System.currentTimeMillis() > initTimeout) {
                noTarget = true;
            }
            return;
        }

        if (firstTarget)
            firstTarget = false;

        Robot.swerve.setChassisSpeeds(camera.getNextTargetSpeeds());
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
    public boolean isFinished() { return stopOnEnd && (noTarget || camera.atTarget()); }
}
