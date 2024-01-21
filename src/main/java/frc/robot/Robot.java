// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.DriveToAprilTagCommand;
import frc.robot.subsystems.SwerveDriveSubsystem;
import frc.robot.util.PhotonCameraModule;
import frc.robot.util.io.*;
import frc.robot.util.joystick.DriveJoystick;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.DriveXboxController;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMode;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

import static frc.robot.Constants.Chassis.*;
//import static frc.robot.Constants.ClimberPresets.*;
import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.LooperConfig.*;
import static frc.robot.Constants.TestPresets.ROTATION_PRESETS;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
    public static final VerbosityLevel verbosity = VerbosityLevel.DEBUG;

    public static PowerDistribution pdh;
    public static DriveXboxController xbox;
    public static DriveJoystick leftStick;
    public static DriveJoystick rightStick;
    public static SwerveDriveSubsystem swerve;
    public static PresetGroup drivePresets;
    public static PhotonCameraModule camera;
    //public static FRCAngledMechanism mechanism;

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // region Initialize AdvantageKit logging. (DO NOT TOUCH)
        Logger.recordMetadata("Project Name", BuildConstants.MAVEN_NAME);
        Logger.recordMetadata("Build Date", BuildConstants.BUILD_DATE);
        Logger.recordMetadata("Git SHA", BuildConstants.GIT_SHA);
        Logger.recordMetadata("Git Date", BuildConstants.GIT_DATE);
        Logger.recordMetadata("Git Branch", BuildConstants.GIT_BRANCH);

        //noinspection RedundantSuppression
        switch (BuildConstants.DIRTY) {
            //noinspection DataFlowIssue
            case 0:
                Logger.recordMetadata("Git Status", "All changes committed");
                break;
            //noinspection DataFlowIssue
            case 1:
                Logger.recordMetadata("Git Status", "Un-committed changes");
                break;
            //noinspection DataFlowIssue
            default:
                Logger.recordMetadata("Git Status", "Unknown");
                break;
        }

        // TODO: setup replay/sim mode!
        //Logger.addDataReceiver(new NT4Publisher());
        //Logger.start(); // start logging!
        // endregion

        boolean useNormalSticks = !RobotBase.isSimulation() ||
                (DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1));

        // Use a PresetGroup to keep the presets synchronized. We don't want one joystick sensitive
        // and the other one non-sensitive.
        drivePresets = new PresetGroup("Drive Presets", PresetMode.PARALLEL);

        if (useNormalSticks) {
            leftStick = new DriveJoystick(
                    LEFT_STICK_ID,  // Left stick ID
                    false,           // Drive X inverted?
                    false,           // Drive Y inverted?
                    false,           // Twist Axis Inverted?
                    DEADBAND,       // Deadband
                    DRIVE_MODES[0], // Primary Drive Mode
                    DRIVE_MODES     // Secondary Drive Modes
            );

            rightStick = new DriveJoystick(
                    RIGHT_STICK_ID,  // Left stick ID
                    false,            // Drive X inverted?
                    false,            // Drive Y inverted?
                    false,            // Twist Axis Inverted?
                    DEADBAND,        // Deadband
                    DRIVE_MODES[0],  // Primary Drive Mode
                    DRIVE_MODES      // Secondary Drive Modes
            );

            drivePresets.add(leftStick);
            drivePresets.add(rightStick);
        }

        xbox = new DriveXboxController(XBOX_CONTROLLER_ID,
                true,
                true,
                true,
                DEADBAND,
                DriveMode.LINEAR_MAP
        );

        if (!useNormalSticks)
            drivePresets.add(xbox); // only add the Xbox Controller if used for driving.

        initLoops();

        pdh = new PowerDistribution();
        /*
        mechanism = new FRCAngledMechanism(
                "Rotation Mechanism",
                GearRatio.from(686, 1),
                new FRCSparkMax(10, CANSparkLowLevel.MotorType.kBrushless, DCMotor.getNEO(1)),
                new PIDConstants(0.1, 0, 0));
        mechanism.registerPresets(ROTATION_PRESETS);
         */

        swerve = new SwerveDriveSubsystem(FL_MODULE, FR_MODULE, BL_MODULE, BR_MODULE);
        camera = new PhotonCameraModule("FrontCamera", Units.inchesToMeters(27), 0);


        BiConsumer<Command, Boolean> logCommandFunction = getCommandActivity();
        CommandScheduler.getInstance().onCommandInitialize(c -> logCommandFunction.accept(c, true));
        CommandScheduler.getInstance().onCommandFinish(c -> logCommandFunction.accept(c, false));
        CommandScheduler.getInstance().onCommandInterrupt(c -> logCommandFunction.accept(c, false));

        // *** IMPORTANT: Call this method at the VERY END of robotInit!!! *** //
        registerAlerts(!useNormalSticks);
        configureBindings(!useNormalSticks);
        // ******************************************************************* //
    }

    private void initLoops() {
        IOManager.initLoop(STRING_PERIODIC_NAME, PERIODIC_INTERVAL);
        IOManager.initLoop(STRING_DASHBOARD_NAME, DASHBOARD_INTERVAL);
        IOManager.initLoop(STRING_ODOMETRY_NAME, ODOMETRY_INTERVAL);
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
                .setCondition(() -> DEBUG_ENABLED)
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
            IOManager.debug(this, "Xbox-only/Simulation mode detected.");
            Robot.swerve.setDefaultCommand(Robot.swerve.runEnd(
                    () -> Robot.swerve.driveRobotRelative(xbox),
                    () -> Robot.swerve.lock())
            );
        } else {
            IOManager.debug(this, "Regular mode detected.");
            Robot.swerve.setDefaultCommand(Robot.swerve.runEnd(
                    () -> Robot.swerve.driveRobotRelative(leftStick, rightStick),
                    () -> Robot.swerve.lock())
            );
        }

        xbox.a().onTrue(Commands.runOnce(() -> ROTATION_PRESETS.setPreset(0)));
        xbox.b().onTrue(Commands.runOnce(() -> ROTATION_PRESETS.setPreset(1)));

//        xbox.a().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(ZERO_POSITION_NAME)));
//        xbox.b().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(FLOOR_CUBE_NAME)));
//        xbox.y().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(HUMAN_STATION_NAME)));
//        xbox.x().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(MID_CONE_NAME)));
//
//        xbox.povDown().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(FLOOR_CONE_NAME)));
//        xbox.povLeft().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(MANUAL_STATION_NAME)));
//        xbox.rightBumper().onTrue(Commands.runOnce(() -> CLIMBER_PRESET_GROUP.setPreset(HIGH_CONE_NAME)));

        if (!xboxOnly) {
            leftStick.button(10).onTrue(Commands.runOnce(() -> drivePresets.nextPreset(true)));
            leftStick.button(12).onTrue(swerve.toggleClosedLoopCommand());
            leftStick.button(11).onTrue(swerve.resetCommand());
            leftStick.trigger().whileTrue(Commands.runEnd(
                    () -> drivePresets.setPreset(2),
                    () -> drivePresets.setPreset(0)
            ));
            leftStick.button(2).whileTrue(Commands.run(() -> swerve.lock()));
            leftStick.button(4).whileTrue(new DriveToAprilTagCommand(
                    new Pose2d(
                            new Translation2d(1, 0),
                            new Rotation2d(0)
                    ), false
            ));
        }
    }

    private static BiConsumer<Command, Boolean> getCommandActivity() {
        Map<String, Integer> commandCounts = new HashMap<>();
        return (Command command, Boolean active) -> {
            String name = command.getName();
            int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
            commandCounts.put(name, count);
            Logger.recordOutput(
                            "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
            Logger.recordOutput("CommandsAll/" + name, count > 0);
        };
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
        IOManager.run();
        // ************************* DO NOT TOUCH ************************* //
    }

    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This method is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        //Robot.mechanism.translateMotor(Robot.xbox.getRobotX());
    }

    @Override
    public void autonomousInit() {
        //new PathPlannerAuto("New Auto").schedule();

    }
}