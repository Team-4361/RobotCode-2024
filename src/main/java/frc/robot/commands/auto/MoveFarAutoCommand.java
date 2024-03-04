package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.DriveToPoseCommand;

import static frc.robot.Constants.TrapArm.ARM_MAX_ROTATION;

public class MoveFarAutoCommand extends SequentialCommandGroup {
    public MoveFarAutoCommand() {
        super(
                Robot.swerve.resetCommand(),
                Commands.runOnce(() -> Robot.arm.setAnglePosition(ARM_MAX_ROTATION)),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                        new Pose2d(
                                                new Translation2d(3.5, 0),
                                                new Rotation2d(0)
                                        )
                        ),
                        new WaitCommand(3)
                ),
                Robot.swerve.runOnce(() -> Robot.swerve.stop())
        );
    }
}
