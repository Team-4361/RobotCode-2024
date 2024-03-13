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
import frc.robot.commands.intake.IntakeNoteCommand;
import frc.robot.commands.shooter.ShootCommand;

import static frc.robot.Constants.TrapFinger.ARM_MAX_ROTATION;

public class TwoNoteMiddleAutoCommand extends SequentialCommandGroup {
    public TwoNoteMiddleAutoCommand() {
        super(
                Robot.swerve.resetCommand(),
                Commands.runOnce(() -> Robot.arm.setAnglePosition(ARM_MAX_ROTATION)),
                new WaitCommand(1),
                new ShootCommand(),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                        new Pose2d(
                                                new Translation2d(1.75, 0),
                                                new Rotation2d(0)
                                        )
                        ),
                        new IntakeNoteCommand(),
                        new WaitCommand(3)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(-0.25, 0),
                                        new Rotation2d(0)
                                )
                        ),
                        new WaitCommand(3)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(-0.25, -1.50),
                                        new Rotation2d(0)
                                )
                        ),
                        new WaitCommand(3)
                ),
                Robot.swerve.runOnce(() -> Robot.swerve.stop()),
                new ShootCommand()
        );
    }
}
