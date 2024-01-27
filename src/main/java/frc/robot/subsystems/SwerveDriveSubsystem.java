package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Robot;
import frc.robot.util.auto.LocalADStarAK;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.swerve.SwerveModule;
import frc.robot.util.swerve.config.GyroIO;
import frc.robot.util.swerve.config.GyroIOInputsAutoLogged;
import frc.robot.util.swerve.config.SwerveModuleIO;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;

import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.AlertConfig.STRING_GYRO_CALIBRATING;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.LooperConfig.*;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveModule}s, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem extends SubsystemBase {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };
    private final SysIdRoutine sysId;

    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveDrivePoseEstimator poseEstimator;

    // Used for keeping the gyro angle at zero when the floor
    // is not PERFECTLY flat. A YAW OFFSET is not required due to integrated
    // functionality in the gyroscope.
    private Rotation2d rollOffset;
    private Rotation2d pitchOffset;

    public static boolean fieldOriented = true;
    public static boolean closedLoop = false;

    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

    /** @return A {@link Command} used to toggle teleoperated closed-loop. */
    public Command toggleClosedLoopCommand() { return Commands.runOnce(() -> closedLoop = !closedLoop); }

    public Command resetCommand() { return Commands.runOnce(this::reset); }

    /** Stops the {@link SwerveDriveSubsystem} from moving. */
    public void stop() {
        setStates(new SwerveModuleState[]
                {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0))
                }, false);
    }

    /**
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules.
     * @param frontLeft  The front left {@link SwerveModule} to use.
     * @param frontRight The front right {@link SwerveModule} to use.
     * @param backLeft   The back left {@link SwerveModule} to use.
     * @param backRight  The back right {@link SwerveModule} to use.
     */
    public SwerveDriveSubsystem(SwerveModule frontLeft,
                                SwerveModule frontRight,
                                SwerveModule backLeft,
                                SwerveModule backRight,
                                GyroIO gyroIO) {
        this.modules = new SwerveModule[]{
                frontLeft,
                frontRight,
                backLeft,
                backRight
        };
        this.gyroIO = gyroIO;
        this.pitchOffset = Rotation2d.fromDegrees(0);
        this.rollOffset = Rotation2d.fromDegrees(0);

        // Define the distances relative to the center of the robot. +X is forward and +Y is left.
        double SWERVE_CHASSIS_SIDE_LENGTH = CHASSIS_MODE.getSideLength();
        this.kinematics = new SwerveDriveKinematics(
                new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2),  // FL
                new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2), // FR
                new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2), // BL
                new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2) // BR
        );
        this.poseEstimator = new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());
        IOManager.getAlert(STRING_GYRO_CALIBRATING, AlertType.WARNING)
                .setCondition(() -> gyroInputs.isCalibrating)
                .setPersistent(false)
                .setDisableDelay(2000)
                .setOneUse(false);

        AutoBuilder.configureHolonomic(
                this::getPose,
                this::reset,
                this::getRobotVelocity,
                this::drive,
                new HolonomicPathFollowerConfig(
                        CHASSIS_MODE.getMaxSpeed(), CHASSIS_BASE_RADIUS, new ReplanningConfig()
                ), () -> {
                    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
                    return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
                },
                this
        );
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    //noinspection ToArrayCallWithZeroLengthArrayArgument
                    Logger.recordOutput(
                            "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose)
        );

        // Configure SysId
        sysId = new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> {
                                    for (SwerveModule module : modules) {
                                        module.runCharacterization(voltage.in(Volts));
                                    }
                                },
                                null,
                                this));
    }

    /**
     * Gets the current field-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current field-relative velocity
     */
    public ChassisSpeeds getFieldVelocity() {
        // ChassisSpeeds has a method to convert from field-relative to robot-relative speeds,
        // but not the reverse.  However, because this transform is a simple rotation, negating the
        // angle
        // given as the robot angle reverses the direction of rotation, and the conversion is reversed.
        return ChassisSpeeds.fromFieldRelativeSpeeds(
                kinematics.toChassisSpeeds(getStates()), getHeading().unaryMinus());
    }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    @SuppressWarnings("RedundantArrayCreation")
    public void periodic() {
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (SwerveModule module : modules) {
            module.update();
        }

        if (DriverStation.isDisabled()) {
            stop();
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = getPositions();
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            moduleDeltas[moduleIndex] =
                    new SwerveModulePosition(
                            modulePositions[moduleIndex].distanceMeters
                                    - lastModulePositions[moduleIndex].distanceMeters,
                            modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }


        // Update gyro angle
        if (gyroInputs.connected) {
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.yawPosition;
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply odometry update
        poseEstimator.update(rawGyroRotation, modulePositions);
    }

    /**
     * Gets the current robot-relative velocity (x, y and omega) of the robot
     *
     * @return A ChassisSpeeds object of the current robot-relative velocity
     */
    public ChassisSpeeds getRobotVelocity() {
        return kinematics.toChassisSpeeds(getStates());
    }

    /** @return The current roll of the {@link Robot} in <b>degrees</b>. */
    public Rotation2d getRoll() { return gyroInputs.rollPosition.minus(rollOffset); }

    /** @return The current pitch of the {@link Robot} in <b>degrees</b>. */
    public Rotation2d getPitch() { return gyroInputs.pitchPosition.minus(pitchOffset); }

    /** @return The current {@link Rotation2d} heading of the {@link Robot}. */
    public Rotation2d getHeading() { return gyroInputs.yawPosition; }

    public void setStates(SwerveModuleState[] states, boolean isClosedLoop) {
        if (states.length != modules.length)
            return;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, CHASSIS_MODE.getMaxSpeed());
        for (int i=0; i<modules.length; i++) {
            modules[i].setState(states[i], isClosedLoop);
        }
    }

    /**
     * Drives the Robot using one Joystick.
     * @param stick The {@link DriveHIDBase} to use.
     */
    public void drive(DriveHIDBase stick) {
        // Calculate the maximum speed based on XY and Twist.
        double xS = stick.getRobotX() * CHASSIS_MODE.getMaxSpeed();
        double yS = stick.getRobotY() * CHASSIS_MODE.getMaxSpeed();
        double tS = stick.getRobotTwist() * CHASSIS_MODE.getMaxSpeed();

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        driveRobotRelative(speeds, closedLoop);
    }

    /**
     * Drives the Robot using two Joysticks (one for XY, one for Twist)
     * @param xyStick The {@link DriveHIDBase} to use for XY translation.
     * @param twistStick The {@link DriveHIDBase} to use for Twist.
     */
    public void drive(DriveHIDBase xyStick, DriveHIDBase twistStick) {
        double xS = xyStick.getRobotX() * CHASSIS_MODE.getMaxSpeed();
        double yS = xyStick.getRobotY() * CHASSIS_MODE.getMaxSpeed();
        double tS = twistStick.getRobotTwist() * CHASSIS_MODE.getMaxSpeed();

        ChassisSpeeds speeds = new ChassisSpeeds(xS, yS, tS);

        if (fieldOriented)
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, getHeading());

        driveRobotRelative(speeds, closedLoop);
    }

    /**
     * Drives the Robot without field orientated using an input {@link ChassisSpeeds}.
     * @param speeds The {@link ChassisSpeeds} to use.
     */
    public void drive(ChassisSpeeds speeds) {setStates(kinematics.toSwerveModuleStates(speeds), false); }

    public void driveRobotRelative(ChassisSpeeds speeds, boolean isClosedLoop) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, CHASSIS_MODE.getMaxSpeed());

        // Send setpoints to modules
        SwerveModuleState[] optimizedSetpointStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedSetpointStates[i] = modules[i].setState(setpointStates[i], true);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedSetpointStates);
    }

    public void lock() {
        setStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(90))
        }, false);
    }

    public void reset(Pose2d pose) {
        rollOffset = gyroInputs.rollPosition;
        pitchOffset = gyroInputs.pitchPosition;
        gyroIO.reset();
        poseEstimator.resetPosition(gyroInputs.yawPosition, getPositions(), pose);
    }

    public void reset() { reset(new Pose2d()); }

    public SwerveModule[] getModules() { return this.modules; }
    public SwerveDriveKinematics getKinematics() { return kinematics; }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[modules.length];
        for (int i=0; i<modules.length; i++) {
            positions[i] = modules[i].getPosition();
        }
        return positions;
    }

    @AutoLogOutput(key = "SwerveStates/Measured")
    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[modules.length];
        for (int i=0; i<modules.length; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    @AutoLogOutput(key = "Odometry/Pose")
    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getRotation() {
        return getPose().getRotation();
    }

    /**
     * Adds a vision measurement to the pose estimator.
     *
     * @param visionPose The pose of the robot as measured by the vision camera.
     * @param timestamp The timestamp of the vision measurement in seconds.
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
        poseEstimator.addVisionMeasurement(visionPose, timestamp);
    }
}
