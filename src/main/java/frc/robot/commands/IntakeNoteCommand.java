package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class IntakeNoteCommand extends Command {
    private long endMillis = 0;

    public IntakeNoteCommand() {
        addRequirements(Robot.intake);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        Robot.intake.start();
        endMillis = 0;
    }

    @Override
    public void execute() {
        if (Robot.intake.hasNote() && endMillis == 0) {
            endMillis = System.currentTimeMillis() + 250;
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
        Robot.intake.stop();
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return endMillis != 0 && System.currentTimeMillis() >= endMillis;
    }
}
