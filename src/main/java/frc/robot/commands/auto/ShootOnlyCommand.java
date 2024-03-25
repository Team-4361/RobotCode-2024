package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.shooter.ShootCommand;

import static frc.robot.Constants.TrapFinger.ARM_MAX_ROTATION;

public class ShootOnlyCommand extends SequentialCommandGroup {
    public ShootOnlyCommand() {
        super(
                Robot.swerve.resetCommand(),
                Commands.runOnce(() -> Robot.arm.setAnglePosition(ARM_MAX_ROTATION)),
                new ShootCommand()
        );
    }
}
