package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ShootCommand extends Command {
    private final double speed;

    public ShootCommand(double targetSpeed) {
        // use "addRequirements" to require the Subsystem inside Robot.
        // example: "Robot.swerve"
        addRequirements(Robot.shooter, Robot.indexer);
        this.speed =targetSpeed;

    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        // Remember to use the "Robot.***" prefix.
        Robot.shooter.moveMotor(speed);

    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        Robot.shooter.moveMotor(speed);


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
       if (interrupted) {
           Robot.shooter.stopMotor();
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
        // remember: returning false here will make your Command run forever, or until
        // a person stops holding down the button. unless we use a sensor in the future,
        // returning false may be a good idea
        return Robot.shooter.getVelocity() >= 4000;
    }
}
