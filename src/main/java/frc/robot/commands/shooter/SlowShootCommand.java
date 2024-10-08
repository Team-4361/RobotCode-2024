package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.Constants.Indexer.SLOW_INDEX_SPEED;
import static frc.robot.Constants.Intake.SLOW_INTAKE_SPEED;
import static frc.robot.Constants.Shooter.*;

public class SlowShootCommand extends Command {
    private long endMillis = System.currentTimeMillis();
    private final boolean timed;

    /**
     * Default constructor.
     */
    public SlowShootCommand(boolean timed) {
        addRequirements(Robot.shooter, Robot.index, Robot.intake);
        this.timed = timed;
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        Robot.index.setTargetSpeed(SLOW_INDEX_SPEED);
        Robot.intake.setTargetSpeed(SLOW_INTAKE_SPEED);
        Robot.shooter.setTargetSpeed(SLOW_SHOOT_SPEED);

        Robot.intake.startNormal();
        Robot.index.startNormal();

        Robot.shooter.setEnabled(true);

        endMillis = System.currentTimeMillis() + 250;
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
        Robot.shooter.setEnabled(false);
        Robot.index.stop();
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
        return timed && System.currentTimeMillis() >= endMillis;
    }
}
