// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.subsystems.*;
import frc.robot.util.auto.PhotonCameraModule;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveJoystick;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.DriveXboxController;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.preset.PresetGroup;

import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.Debug.DEBUG_LOGGING_ENABLED;
import static frc.robot.Constants.Presets.TRAP_ARM_PRESETS;
import static frc.robot.Constants.Presets.TRAP_PRESET_GROUP;
import static frc.robot.util.math.GlobalUtils.deadband;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static PowerDistribution pdh;
    public static DriveXboxController xbox;
    public static DriveJoystick leftStick;
    public static DriveJoystick rightStick;
    public static SwerveDriveSubsystem swerve;
    public static PresetGroup drivePresets;
    public static PhotonCameraModule frontCamera;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static IndexSubsystem index;
    public static TrapWristSubsystem wrist;
    public static ClimberSubsystem climber;
    public static TrapArmSubsystem arm;


    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        boolean useNormalSticks = true;
        // Use a PresetGroup to keep the presets synchronized. We don't want one joystick sensitive
        // and the other one non-sensitive.
        drivePresets = new PresetGroup("Drive Presets");

        //noinspection ConstantValue
        if (useNormalSticks) {
            leftStick = new DriveJoystick(
                    LEFT_STICK_ID,  // Left stick ID
                    false,          // Drive X inverted?
                    false,          // Drive Y inverted?
                    false,          // Twist Axis Inverted?
                    DEAD_ZONE,      // Dead-band
                    DRIVE_MODES[0], // Primary Drive Mode
                    DRIVE_MODES     // Secondary Drive Modes
            );

            rightStick = new DriveJoystick(
                    RIGHT_STICK_ID, // Right stick ID
                    false,          // Drive X inverted?
                    false,          // Drive Y inverted?
                    false,          // Twist Axis Inverted?
                    DEAD_ZONE,      // Dead-band
                    DRIVE_MODES[0], // Primary Drive Mode
                    DRIVE_MODES     // Secondary Drive Modes
            );

            drivePresets.add(leftStick);
            drivePresets.add(rightStick);
        }

        xbox = new DriveXboxController(XBOX_CONTROLLER_ID,
                true,
                true,
                true,
                DEAD_ZONE,
                DriveMode.LINEAR_MAP
        );

        if (!useNormalSticks)
            drivePresets.add(xbox); // only add the Xbox Controller if used for driving.

        pdh = new PowerDistribution(34, ModuleType.kRev);
        intake = new IntakeSubsystem();
        shooter = new ShooterSubsystem();
        index = new IndexSubsystem();
        wrist = new TrapWristSubsystem();
        climber = new ClimberSubsystem();
        arm = new TrapArmSubsystem();
        frontCamera = new PhotonCameraModule(
                "FrontCamera",
                Units.inchesToMeters(27),
                0
        );

        SwerveDriveSubsystem.initParser();
        swerve = new SwerveDriveSubsystem();

        // *** IMPORTANT: Call this method at the VERY END of robotInit!!! *** //
        registerAlerts(!useNormalSticks);
        configureBindings(!useNormalSticks);
        // ******************************************************************* //
    }

    private void registerAlerts(boolean xboxOnly) {
        IOManager.getAlert("Idle Voltage Low", AlertType.WARNING)
                .setCondition(() -> Robot.pdh.getVoltage() < 12 && Robot.pdh.getTotalCurrent() <= 2.5);

        DriverStation.silenceJoystickConnectionWarning(true);
        IOManager.getAlert("Joystick Not Connected", AlertType.ERROR)
                .setCondition(() ->
                        !DriverStation.isJoystickConnected(0)
                                || !DriverStation.isJoystickConnected(1)
                                || !DriverStation.isJoystickConnected(2));

        IOManager.getAlert("Debug mode enabled", AlertType.INFO)
                .setCondition(() -> DEBUG_LOGGING_ENABLED)
                .setPersistent(true);

        if (!xboxOnly) {
            IOManager.getAlert("Slow mode enabled", AlertType.INFO)
                    .setCondition(() ->
                            leftStick.getPresetName().equalsIgnoreCase("Slow Mode") ||
                                    rightStick.getPresetName().equalsIgnoreCase("Slow Mode"))
                    .setPersistent(false);
        }
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings(boolean xboxOnly) {
        if (xboxOnly) {
            IOManager.info(this, "Xbox-only/Simulation mode detected.");
            Robot.swerve.setDefaultCommand(Robot.swerve.runEnd(
                    () -> Robot.swerve.drive(xbox),
                    () -> Robot.swerve.lockPose())
            );
        } else {
            IOManager.info(this, "Regular mode detected.");
            Robot.swerve.setDefaultCommand(Robot.swerve.runEnd(
                    () -> Robot.swerve.drive(leftStick, rightStick),
                    () -> Robot.swerve.lockPose())
            );
        }

        if (!xboxOnly) {
            leftStick.button(10).onTrue(Commands.runOnce(() -> drivePresets.nextPreset(true)));
            leftStick.button(11).onTrue(swerve.resetCommand());
            leftStick.button(12).onTrue(swerve.toggleFieldOrientedCommand());
            leftStick.trigger().whileTrue(Commands.runEnd(
                    () -> drivePresets.setPreset(2),
                    () -> drivePresets.setPreset(0)
            ));
            leftStick.button(2).whileTrue(Commands.run(() -> swerve.lockPose()));
            leftStick.button(4).whileTrue(new DriveToAprilTagCommand(
                    new Pose2d(
                            new Translation2d(2, 0),
                            new Rotation2d(0)
                    ), 27, 7, false
            ));
        }

        xbox.b().whileTrue(new IntakeNoteCommand());
        xbox.a().onTrue(new ShootCommand());

        /*
        xbox.rightTrigger().whileTrue(Robot.climber.runEnd(
                () -> {
                    Robot.climber.moveLeftUp();
                    Robot.climber.moveRightUp();
                },
                () -> {
                    Robot.climber.stopLeft();
                    Robot.climber.stopRight();
                }
        ));
        xbox.leftTrigger().whileTrue(Robot.climber.runEnd(
                () -> {
                    Robot.climber.moveLeftDown();
                    Robot.climber.moveRightDown();
                },
                () -> {
                    Robot.climber.stopLeft();
                    Robot.climber.stopRight();
                }
        ));

         */

        xbox.leftTrigger().whileTrue(Robot.climber.runEnd(
                () -> Robot.climber.moveLeftUp(),
                () -> Robot.climber.stopLeft()
        ));
        xbox.rightTrigger().whileTrue(Robot.climber.runEnd(
                () -> Robot.climber.moveRightUp(),
                () -> Robot.climber.stopRight()
        ));
        xbox.leftBumper().whileTrue(Robot.climber.runEnd(
                () -> Robot.climber.moveLeftDown(),
                () -> Robot.climber.stopLeft()
        ));
        xbox.rightBumper().whileTrue(Robot.climber.runEnd(
                () -> Robot.climber.moveRightDown(),
                () -> Robot.climber.stopRight()
        ));
    }


    /**
     * This method is called every 20 ms, no matter the mode. Use this for items like diagnostics
     * that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic methods, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // ************************* DO NOT TOUCH ************************* //

        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();
        Robot.frontCamera.update();
        IOManager.run();
    }

    @Override
    public void autonomousInit() {
        new PathPlannerAuto("New Auto").schedule();
    }

    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void teleopInit() { CommandScheduler.getInstance().cancelAll(); }
}