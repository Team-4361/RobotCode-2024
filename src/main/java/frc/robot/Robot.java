// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.climber.LeftClimbDownCommand;
import frc.robot.commands.climber.RightClimbDownCommand;
import frc.robot.commands.intake.AmpCommand;
import frc.robot.commands.intake.IntakeNoteCommand;
import frc.robot.commands.intake.OuttakeNoteCommand;
import frc.robot.commands.shooter.ShootCommand;
import frc.robot.commands.shooter.SlowShootCommand;
import frc.robot.subsystems.*;
import frc.robot.util.auto.PhotonCameraModule;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;

import java.util.Optional;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Chassis.SIDE_LENGTH_METERS;
import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.Power.POWER_CAN_ID;
import static frc.robot.Constants.Power.POWER_MODULE_TYPE;
import static frc.robot.Constants.Presets.TRAP_PRESET_GROUP;
import static frc.robot.Constants.Shooter.SHOOT_SPEED;
import static frc.robot.Constants.FrontCamera.*;
import static frc.robot.Constants.ShooterCamera.SHOOTER_PIPELINES;
import static frc.robot.Constants.ShooterCamera.SHOOT_CAMERA_TRANSFORM;
import static frc.robot.Constants.Systems.FRONT_CAMERA;
import static frc.robot.Constants.Systems.SHOOTER_CAMERA;
import static frc.robot.subsystems.ClimberSubsystem.MoveDirection.UP;
import static frc.robot.util.math.GlobalUtils.deadband;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static CommandXboxController xbox;
    public static CommandJoystick leftStick;
    public static CommandJoystick rightStick;

    public static PowerDistribution pdh;
    public static SwerveDriveSubsystem swerve;
    public static PhotonCameraModule frontCamera;
    public static PhotonCameraModule shooterCamera;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static IndexSubsystem index;
    public static ClimberSubsystem climber;
    public static FingerSubsystem arm;

    private SendableChooser<Command> autoChooser;

    private void startDriverCamera() {
        int width = 360;
        int height = 240;
        Thread camThread = new Thread(
                () -> {
                    try (UsbCamera camera = CameraServer.startAutomaticCapture()) {
                        if (!RobotBase.isSimulation()) {
                            camera.setResolution(width, height);
                            camera.setFPS(60);
                        }

                        CvSink cvSink = CameraServer.getVideo();
                        CvSource outputStream = CameraServer.putVideo("Front Camera", width, height);
                        Mat mat = new Mat();
                        double thickness = 4.0;

                        while (!Thread.currentThread().isInterrupted()) {
                            if (cvSink.grabFrame(mat) == 0) {
                                outputStream.notifyError(cvSink.getError());
                                continue;
                            }
                            Imgproc.line(
                                    mat,
                                    new Point((width/2.0)-125-(thickness*5), height),
                                    new Point((width/2.0)-(thickness*5), height-100),
                                    new Scalar(0, 0, 0),
                                    (int)thickness
                            );
                            Imgproc.line(
                                    mat,
                                    new Point((width/2.0)+125-(thickness*5), height),
                                    new Point((width/2.0)-(thickness*5), height-100),
                                    new Scalar(0, 0, 0),
                                    (int)thickness
                            );
                            outputStream.putFrame(mat);
                        }

                    } catch (Exception ex) {
                        new Alert("Failed to configure operator camera!", AlertType.ERROR).set(true);
                        Thread.currentThread().interrupt();
                    }
                }
        );
        camThread.setDaemon(true);
        camThread.start();
    }

    /**
     * This method is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        if (!RobotBase.isSimulation())
            startDriverCamera();

        leftStick = new CommandJoystick(LEFT_STICK_ID);
        rightStick = new CommandJoystick(RIGHT_STICK_ID);
        xbox = new CommandXboxController(XBOX_CONTROLLER_ID);
        pdh = new PowerDistribution(POWER_CAN_ID, POWER_MODULE_TYPE);

        intake = new IntakeSubsystem();
        shooter = new ShooterSubsystem();
        index = new IndexSubsystem();
        climber = new ClimberSubsystem();
        arm = new FingerSubsystem();

        frontCamera = new PhotonCameraModule(FRONT_CAMERA, FRONT_CAMERA_TRANSFORM, FRONT_PIPELINES);
        shooterCamera = new PhotonCameraModule(SHOOTER_CAMERA, SHOOT_CAMERA_TRANSFORM, SHOOTER_PIPELINES);
        swerve = new SwerveDriveSubsystem();

        NamedCommands.registerCommand("IntakeCommand", new IntakeNoteCommand());
        NamedCommands.registerCommand("ShootCommand", new ShootCommand());
        NamedCommands.registerCommand("FirstShootCommand", new ShootCommand(1));
        NamedCommands.registerCommand("AmpUpCommand", new AmpCommand());
        NamedCommands.registerCommand("AmpDownCommand", Commands.runOnce(() -> TRAP_PRESET_GROUP.setPreset(0)));
        NamedCommands.registerCommand("ResetGyroCommand", Commands.runOnce(() -> {
            Robot.swerve.reset();
            Robot.swerve.hasResetGyro = true; // force it in auto
        }));

        AutoBuilder.configureHolonomic(
                () -> Robot.swerve.getPose(),
                (pose) -> Robot.swerve.reset(pose),
                () -> Robot.swerve.getRobotVelocity(),
                (speeds) -> Robot.swerve.setChassisSpeeds(speeds),
                new HolonomicPathFollowerConfig(
                        AUTO_DRIVE_PID,
                        AUTO_TURN_PID,
                        MAX_SPEED_MPS,
                        Math.hypot(SIDE_LENGTH_METERS/2, SIDE_LENGTH_METERS/2),
                        new ReplanningConfig()
                ), () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                Robot.swerve
        );

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public static Command resetAllCommand() {
        return Commands.runOnce(() -> {
            Robot.swerve.reset();
            //Robot.climber.reset();
            Robot.arm.reset();
            //Robot.wrist.reset();
        });
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
    private void configureBindings() {

        Command teleopFlightDriveCommand = Robot.swerve.driveCommand(
                () -> -deadband(leftStick.getY()), // +X forward | -X reverse
                () -> -deadband(leftStick.getX()), // +Y left | -Y right
                () -> -deadband(rightStick.getTwist())); // CCW positive

        Command teleopXboxDriveCommand = Robot.swerve.driveCommand(
                () -> -deadband(xbox.getLeftY()),
                () -> -deadband(xbox.getLeftX()),
                () -> -deadband(xbox.getRightX())
        );

        //Robot.swerve.setDefaultCommand(new DriveCommand()); // will be set this way on real robot.
        Robot.swerve.setDefaultCommand(teleopFlightDriveCommand);

        leftStick.button(11).onTrue(Robot.resetAllCommand());
        leftStick.trigger().whileTrue(swerve.toggleSlowModeCommand());

        leftStick.button(4).whileTrue(Commands.runEnd(
                () -> Robot.swerve.fieldOriented = false,
                () -> Robot.swerve.fieldOriented = true
        ));
        leftStick.button(3).whileTrue(new DriveTargetCommand(frontCamera, 0,
                new Transform2d(
                        new Translation2d(0.2, 0),
                        new Rotation2d(0)
                ), true
        ));
        leftStick.button(5).whileTrue(new DriveTargetCommand(shooterCamera, 0,
                new Transform2d(
                        new Translation2d(1, 0),
                        new Rotation2d(0)
                ), true
        ));

        xbox.b().whileTrue(new IntakeNoteCommand());
        xbox.a().onTrue(new ShootCommand());
        xbox.y().whileTrue(new SlowShootCommand(false));
        xbox.x().whileTrue(new OuttakeNoteCommand());

        xbox.leftBumper().whileTrue(new LeftClimbDownCommand());
        xbox.rightBumper().whileTrue(new RightClimbDownCommand());

        xbox.back().whileTrue(Commands.runEnd(
                () -> Robot.climber.moveLeft(UP),
                () -> Robot.climber.stopLeft()
        ));
        xbox.start().whileTrue(Commands.runEnd(
                () -> Robot.climber.moveRight(UP),
                () -> Robot.climber.stopRight()
        ));

        xbox.rightTrigger().whileTrue(Commands.runEnd(
                () -> {Robot.shooter.setTargetSpeed(SHOOT_SPEED); Robot.shooter.setEnabled(true);},
                () -> Robot.shooter.setEnabled(false)
        ));

        xbox.povDown().onTrue(Commands.runOnce(() -> TRAP_PRESET_GROUP.setPreset(0)));
        xbox.povUp().onTrue(new AmpCommand());
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
    }

    @Override public void autonomousInit() { autoChooser.getSelected().schedule(); }
    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void teleopInit() { CommandScheduler.getInstance().cancelAll(); }
}