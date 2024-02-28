package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;

import static frc.robot.Constants.TrapArm.ARM_SERVO_MAX_MM;

public class SimpleAutoCommand extends SequentialCommandGroup {
    public SimpleAutoCommand() {
        super(
                Robot.swerve.resetCommand(),
                Commands.runOnce(() -> Robot.arm.setAnglePosition(ARM_SERVO_MAX_MM)),
                new ShootCommand(),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                        new Pose2d(
                                                new Translation2d(-1.75, 0),
                                                new Rotation2d(0)
                                        )
                        ),
                        new IntakeNoteCommand(),
                        new WaitCommand(3)
                ),
                new ParallelRaceGroup(
                        new DriveToPoseCommand(
                                new Pose2d(
                                        new Translation2d(0, 0),
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
