package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.Constants.Shooter.SHOOT_END_DELAY_MS;

public class ShootCommand extends Command {
    private long endMillis = 0;
    private long feedMillis = 0;

    /**
     * Default constructor.
     */
    public ShootCommand() { addRequirements(Robot.shooter, Robot.index, Robot.intake); }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        endMillis = 0;
        feedMillis = System.currentTimeMillis() + 2000;
        Robot.shooter.start();
        Robot.index.stop();
        Robot.intake.stop();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (System.currentTimeMillis() >= feedMillis) {
            Robot.index.start();
            Robot.intake.start();
            if (endMillis == 0)
                endMillis = System.currentTimeMillis() + SHOOT_END_DELAY_MS;
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
        Robot.shooter.stop();
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
        return endMillis != 0 && System.currentTimeMillis() >= endMillis;
    }
}
