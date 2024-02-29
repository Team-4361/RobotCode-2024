package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

import static frc.robot.Constants.Indexer.SLOW_INDEX_SPEED;
import static frc.robot.Constants.Intake.SLOW_INTAKE_SPEED;
import static frc.robot.Constants.Shooter.SLOW_SHOOT_SPEED;

public class SlowShootCommand extends Command {

    public SlowShootCommand() {
        addRequirements(Robot.index, Robot.intake, Robot.shooter);
    }

    @Override
    public void initialize() {
        Robot.index.setTargetSpeed(SLOW_INDEX_SPEED);
        Robot.intake.setTargetSpeed(SLOW_INTAKE_SPEED);
        Robot.shooter.setTargetSpeed(SLOW_SHOOT_SPEED);

        Robot.index.start();
        Robot.intake.startNormal();
        Robot.shooter.start();
    }

    @Override
    public void end(boolean interrupted) {
        Robot.index.stop();
        Robot.intake.stop();
        Robot.shooter.stop();
    }

    @Override
    public boolean isFinished() {
        return false; // TODO: make it time based
    }
}
