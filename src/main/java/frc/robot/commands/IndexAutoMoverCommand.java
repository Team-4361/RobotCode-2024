package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.Constants.INDEX_SPEED;
import static frc.robot.Constants.INTAKE_SPEED;

public class IndexAutoMoverCommand extends Command {

    public IndexAutoMoverCommand() {
        addRequirements(Robot.indexer);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
      Robot.indexer.moveIndexer(INDEX_SPEED);
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        Robot.indexer.moveIndexer(INDEX_SPEED);

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

        Robot.indexer.stopIndexer();
        Robot.shooter.stopMotor();

    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        if (!Robot.indexer.hasNote() || Robot.shooter.isNoPower()){
            return true;
        }
        else{
            return false;
        }
    }
}
