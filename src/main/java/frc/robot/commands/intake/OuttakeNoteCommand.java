package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.Constants.Indexer.INDEX_SPEED;
import static frc.robot.Constants.Intake.INTAKE_SPEED;

public class OuttakeNoteCommand extends Command {

    public OuttakeNoteCommand() {
        addRequirements(Robot.intake, Robot.index);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        Robot.intake.setTargetSpeed(INTAKE_SPEED);
        Robot.index.setTargetSpeed(INDEX_SPEED);

        Robot.intake.startReverse();
        Robot.index.startReverse();
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
        Robot.index.stop();
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return false;
    }
}
