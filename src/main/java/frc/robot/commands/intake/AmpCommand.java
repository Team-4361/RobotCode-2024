package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.shooter.SlowShootCommand;

import static frc.robot.Constants.Presets.TRAP_PRESET_GROUP;

public class AmpCommand extends SequentialCommandGroup {
    public AmpCommand() {
        super(
            Commands.runOnce(() -> Robot.arm.setMaxPower(0.25)),
            TRAP_PRESET_GROUP.setPresetCommand(1),
            new SlowShootCommand(true),
            new WaitCommand(1),
            Commands.runOnce(() -> Robot.arm.setMaxPower(0.60)),
               TRAP_PRESET_GROUP.setPresetCommand(2)
        );
    }
}
