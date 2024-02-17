package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimbDownCommand extends Command {
    public ClimbDownCommand() {
        addRequirements(Robot.climber);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        Robot.climber.moveLeftDown();
        Robot.climber.moveRightDown();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (Robot.climber.isLeftRetracted()) {
            Robot.climber.stopLeft();
        }
        if (Robot.climber.isRightRetracted()) {
            Robot.climber.stopRight();
        }
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
        Robot.climber.stopLeft();
        Robot.climber.stopRight();
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return Robot.climber.isLeftRetracted() && Robot.climber.isRightRetracted();
    }
}
