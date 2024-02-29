// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.util.PixelFormat;
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
import frc.robot.subsystems.*;
import frc.robot.util.auto.AprilTagName;
import frc.robot.util.auto.PhotonCameraModule;
import frc.robot.util.math.GlobalUtils;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import swervelib.telemetry.Alert;
import swervelib.telemetry.Alert.AlertType;

import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.Power.POWER_CAN_ID;
import static frc.robot.Constants.Power.POWER_MODULE_TYPE;
import static frc.robot.Constants.ShooterCamera.*;
import static frc.robot.util.math.GlobalUtils.deadband;


/**
 * The VM is configured to automatically run this class, and to call the methods corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
    public static PowerDistribution pdh;
    public static CommandXboxController xbox;
    public static CommandJoystick leftStick;
    public static CommandJoystick rightStick;
    public static SwerveDriveSubsystem swerve;
    public static PhotonCameraModule shooterCamera;
    public static ShooterSubsystem shooter;
    public static IntakeSubsystem intake;
    public static IndexSubsystem index;
    public static TrapWristSubsystem wrist;
    public static ClimberSubsystem climber;
    public static TrapArmSubsystem arm;

    private SendableChooser<Command> autoChooser;


    @SuppressWarnings("resource")
    private void startDriverCamera() {
        int width = 360;
        int height = 240;
        Thread camThread = new Thread(
                () -> {
                    try {
                        UsbCamera camera = CameraServer.startAutomaticCapture();
                        if (!RobotBase.isSimulation()) {
                            camera.setResolution(width, height);
                            camera.setFPS(30);
                            //camera.setVideoMode(new VideoMode(PixelFormat.kGray, width, height, 60));
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
        wrist = new TrapWristSubsystem();
        climber = new ClimberSubsystem();
        arm = new TrapArmSubsystem();

        shooterCamera = new PhotonCameraModule(
                SHOOT_CAMERA_NAME,
                SHOOT_CAMERA_HEIGHT_METERS,
                SHOOT_CAMERA_PITCH_DEGREES
        );

        SwerveDriveSubsystem.initParser();
        swerve = new SwerveDriveSubsystem();

        autoChooser = new SendableChooser<>();
        autoChooser.addOption("None", Commands.runOnce(() -> Robot.swerve.reset()));
        autoChooser.addOption("Two Note Shoot", new TwoNoteAutoCommand());
        autoChooser.setDefaultOption("None", Commands.runOnce(() -> Robot.swerve.reset()));

        SmartDashboard.putData("Auto Chooser", autoChooser);
        configureBindings();
    }

    public static Command resetAllCommand() {
        return Commands.runOnce(() -> {
            Robot.swerve.reset();
            Robot.climber.reset();
            Robot.arm.reset();
            Robot.wrist.reset();
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

        Robot.swerve.setDefaultCommand(teleopFlightDriveCommand); // will be set this way on real robot.
        //Robot.swerve.setDefaultCommand(teleopXboxDriveCommand);

        leftStick.button(11).onTrue(Robot.resetAllCommand());
        leftStick.button(12).onTrue(swerve.toggleFieldOrientedCommand());
        leftStick.trigger().whileTrue(swerve.toggleSlowModeCommand());

        leftStick.button(4).whileTrue(new DriveToAprilTagCommand(
                new Pose2d(
                        new Translation2d(2, 0),
                        new Rotation2d(0)
                ), 27, AprilTagName.getAllianceID("Shooter"), false
        ));

        xbox.b().whileTrue(new IntakeNoteCommand());
        xbox.a().onTrue(new ShootCommand());
        xbox.y().whileTrue(Robot.intake.runEnd(
                () -> Robot.intake.startReverse(),
                () -> Robot.intake.stop()
        ));

        // Xbox Y --> reverse intake (hold)
        // each bumper controls each side of the climber
        // Xbox X --> auto grab note
        // Xbox left-dpad + right-dpad --> place the note


        /* TODO: add proper button bindings
        xbox.leftTrigger().whileTrue(Commands.runEnd(
                () -> Robot.climber.moveLeftUp(),
                () -> Robot.climber.stopLeft()
        ));
        xbox.rightTrigger().whileTrue(Commands.runEnd(
                () -> Robot.climber.moveRightUp(),
                () -> Robot.climber.stopRight()
        ));
        xbox.leftBumper().whileTrue(Commands.runEnd(
                () -> Robot.climber.moveLeftDown(),
                () -> Robot.climber.stopLeft()
        ));
        xbox.rightBumper().whileTrue(Commands.runEnd(
                () -> Robot.climber.moveRightDown(),
                () -> Robot.climber.stopRight()
        ));
         */

        xbox.leftBumper()
                .and(xbox.rightBumper())
                .whileTrue(new ClimbDownCommand());
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
        Robot.shooterCamera.update();

        Robot.arm.setExtensionSpeed(deadband(-Robot.xbox.getLeftY()));
        Robot.arm.setAngleSpeed(deadband(-Robot.xbox.getRightY()));

        Robot.wrist.translateWrist(
                GlobalUtils.getDualSpeed(
                        Robot.xbox.getLeftTriggerAxis(),
                        Robot.xbox.getRightTriggerAxis()
                )/3
        );
    }

    @Override public void autonomousInit() { autoChooser.getSelected().schedule(); }
    @Override public void disabledInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void testInit() { CommandScheduler.getInstance().cancelAll(); }
    @Override public void teleopInit() { CommandScheduler.getInstance().cancelAll(); }
}