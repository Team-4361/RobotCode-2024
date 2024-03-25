package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import java.util.Optional;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;
import static frc.robot.util.math.GlobalUtils.deadband;

public class DriveCommand extends Command {
    private long nextCheck;
    private boolean flipXY = true;

    public DriveCommand() {
        addRequirements(Robot.swerve);
    }

    /**
     * The initial subroutine of a command. Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {
        nextCheck = System.currentTimeMillis();
    }

    /**
     * The main body of a command. Called repeatedly while the command is scheduled.
     */
    @Override
    public void execute() {
        if (System.currentTimeMillis() >= nextCheck) {
            // Attempt to pull the Alliance from the driver station.
            Optional<Alliance> color = DriverStation.getAlliance();
            color.ifPresent(alliance -> flipXY = (alliance == Blue || Robot.swerve.hasResetGyro));

            nextCheck = System.currentTimeMillis() + 1000;
        }

        double tX = Robot.leftStick.getY();
        double tY = Robot.leftStick.getX();
        double tOmega = -Robot.rightStick.getTwist();

        if (flipXY) {
            tX *= -1;
            tY *= -1;
        }

        Robot.swerve.rawDrive(
                new Translation2d(
                        Math.pow(tX, 3) * Robot.swerve.getMaximumVelocity(),
                        Math.pow(tY, 3) * Robot.swerve.getMaximumVelocity()
                ),
                Math.pow(tOmega, 3) * Robot.swerve.getMaximumAngularVelocity(),
                Robot.swerve.fieldOriented,
                false
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
        super.end(interrupted);
    }

    /**
     * Whether the command has finished. Once a command finishes, the scheduler will call its end()
     * method and un-schedule it.
     *
     * @return whether the command has finished.
     */
    @Override
    public boolean isFinished() {
        return super.isFinished();
    }
}
