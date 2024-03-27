package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.Constants.Climber.CLIMBER_SENSOR_DELAY_MS;
import static frc.robot.subsystems.ClimberSubsystem.MoveDirection.DOWN;

public class RightClimbDownCommand extends Command {
    public RightClimbDownCommand() { }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override public void initialize() { Robot.climber.moveRight(DOWN); }

    /**
     * The action to take when the command ends. Called when either the command finishes normally, or
     * when it interrupted/canceled.
     *
     * <p>Do not schedule commands here that share requirements with this command. Use {@link
     * #andThen(Command...)} instead.
     *
     * @param interrupted whether the command was interrupted/canceled
     */
    @Override public void end(boolean interrupted) { Robot.climber.stopRight(); }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return Robot.climber.isRightRetracted() &&
                Robot.climber.getRightActivatedDuration() >= CLIMBER_SENSOR_DELAY_MS;
    }
}
