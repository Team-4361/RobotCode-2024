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

import static frc.robot.Constants.TrapArm.ARM_MAX_ROTATION;

public class ThreeNoteAutoCommand extends SequentialCommandGroup {

    public ThreeNoteAutoCommand() {
        super(
                Robot.swerve.resetCommand(),
                new ShootCommand(),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(2.5, 0),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new IntakeNoteCommand(),
                        new WaitCommand(1.5)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(0, 0),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5)
                ),
                new ShootCommand(),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(1, 0),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(1, 1.55),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(2.5, 1.55),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5),
                        new IntakeNoteCommand()
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(1.5, 1.55),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(1.5, 0),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(0, 0),
                                        Rotation2d.fromDegrees(0)
                                )
                        ),
                        new WaitCommand(1.5)
                ),
                new ShootCommand(),
                Commands.runOnce(() -> Robot.swerve.reset()),
                Robot.swerve.runOnce(() -> Robot.swerve.stop())
        );
    }
}
