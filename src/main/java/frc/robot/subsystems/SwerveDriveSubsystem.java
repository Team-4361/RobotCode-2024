package frc.robot.subsystems;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.auto.LocalADStarAK;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveHIDBase;
import frc.robot.util.swerve.SwerveModuleBase;
import frc.robot.util.swerve.SwerveModuleCAN;
import frc.robot.util.swerve.SwerveModuleMAG;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.Optional;

import static frc.robot.Constants.AlertConfig.STRING_GYRO_CALIBRATING;
import static frc.robot.Constants.Chassis.*;

/**
 * This {@link SwerveDriveSubsystem} is designed to be used for controlling the {@link SwerveModuleBase}s, and utilizing
 * an {@link AHRS} gyroscope to provide the field-relating driving a robot needs. This is also useful for debugging
 * purposes (or for simple autonomous) as it allows driving in a specific direction.
 *
 * @since 0.0.0
 * @author Eric Gold
 */
public class SwerveDriveSubsystem extends SubsystemBase implements LoggableInputs {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModuleBase[] modules;
    private final AHRS gyro;
    private final SwerveModulePosition[] lastModulePositions = new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
    };

    public boolean connected = false;
    public Rotation2d yawPosition = new Rotation2d();
    public double yawVelocityRadPerSec = 0.0;
    public Rotation2d pitchPosition = new Rotation2d();
    public Rotation2d rollPosition = new Rotation2d();
    public boolean isCalibrating = false;

    private Rotation2d rawGyroRotation = new Rotation2d();
    private final SwerveDrivePoseEstimator poseEstimator;

    // Used for keeping the gyro angle at zero when the floor
    // is not PERFECTLY flat. A YAW OFFSET is not required due to integrated
    // functionality in the gyroscope.
    private Rotation2d rollOffset;
    private Rotation2d pitchOffset;

    public static boolean fieldOriented = true;

    /** @return A {@link Command} used to toggle teleoperated field-oriented. */
    public Command toggleFieldOrientedCommand() { return Commands.runOnce(() -> fieldOriented = !fieldOriented); }

    public Command resetCommand() { return Commands.runOnce(this::reset); }

    /** Stops the {@link SwerveDriveSubsystem} from moving. */
    public void stop() {
        setStates(new SwerveModuleState[]
                {
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0)),
                        new SwerveModuleState(0, Rotation2d.fromDegrees(0))
                });
    }

    /**
     * Constructs a new {@link SwerveDriveSubsystem} with the specified modules.
     */
    public SwerveDriveSubsystem() {
        if (CHASSIS_MODE.usingMagEncoders()) {
            this.modules = new SwerveModuleBase[]{
                    new SwerveModuleMAG("FL", CHASSIS_MODE.getFLModule()),
                    new SwerveModuleMAG("FR", CHASSIS_MODE.getFRModule()),
                    new SwerveModuleMAG("BL", CHASSIS_MODE.getBLModule()),
                    new SwerveModuleMAG("BR", CHASSIS_MODE.getBRModule()),
            };
        } else {
            this.modules = new SwerveModuleBase[]{
                    new SwerveModuleCAN("FL", CHASSIS_MODE.getFLModule()),
                    new SwerveModuleCAN("FR", CHASSIS_MODE.getFRModule()),
                    new SwerveModuleCAN("BL", CHASSIS_MODE.getBLModule()),
                    new SwerveModuleCAN("BR", CHASSIS_MODE.getBRModule()),
            };
        }

        this.gyro = new AHRS(SPI.Port.kMXP);
        gyro.reset();

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

        this.poseEstimator = new SwerveDrivePoseEstimator(
                kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

        IOManager.getAlert(STRING_GYRO_CALIBRATING, AlertType.WARNING)
                .setCondition(() -> isCalibrating)
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
        connected = gyro.isConnected();
        yawPosition = gyro.getRotation2d();
        yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
        pitchPosition = Rotation2d.fromDegrees(gyro.getPitch());
        rollPosition = Rotation2d.fromDegrees(gyro.getRoll());
        isCalibrating = gyro.isCalibrating();

        Logger.processInputs("Drive/Gyro", this);

        for (SwerveModuleBase module : modules)
            module.update();

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
        if (connected) {
            // Use the real gyro angle
            rawGyroRotation = yawPosition;
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
    public Rotation2d getRoll() { return rollPosition.minus(rollOffset); }

    /** @return The current pitch of the {@link Robot} in <b>degrees</b>. */
    public Rotation2d getPitch() { return pitchPosition.minus(pitchOffset); }

    /** @return The current {@link Rotation2d} heading of the {@link Robot}. */
    public Rotation2d getHeading() { return yawPosition; }

    public void setStates(SwerveModuleState[] states) {
        if (states.length != modules.length)
            return;

        SwerveDriveKinematics.desaturateWheelSpeeds(states, CHASSIS_MODE.getMaxSpeed());
        for (int i=0; i<modules.length; i++) {
            modules[i].setState(states[i]);
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

        driveRobotRelative(speeds);
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

        driveRobotRelative(speeds);
    }

    /**
     * Drives the Robot without field orientated using an input {@link ChassisSpeeds}.
     * @param speeds The {@link ChassisSpeeds} to use.
     */
    public void drive(ChassisSpeeds speeds) { setStates(kinematics.toSwerveModuleStates(speeds)); }

    public void driveRobotRelative(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] states = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(states, CHASSIS_MODE.getMaxSpeed());

        // Send setpoints to modules
        SwerveModuleState[] optimizedStates = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            // The module returns the optimized state, useful for logging
            optimizedStates[i] = modules[i].setState(states[i]);
        }

        // Log setpoint states
        Logger.recordOutput("SwerveStates/Setpoints", states);
        Logger.recordOutput("SwerveStates/SetpointsOptimized", optimizedStates);
    }

    public void lock() {
        setStates(new SwerveModuleState[]{
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))
        });
    }

    public void reset(Pose2d pose) {
        rollOffset = rollPosition;
        pitchOffset = pitchPosition;
        Constants.runIfNotReplay(gyro::reset);
        poseEstimator.resetPosition(yawPosition, getPositions(), pose);
    }

    public void reset() { reset(new Pose2d()); }

    public SwerveModuleBase[] getModules() { return this.modules; }
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

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("Connected", this.connected);
        table.put("YawPosition", this.yawPosition);
        table.put("YawVelocityRadPerSec", this.yawVelocityRadPerSec);
        table.put("PitchPosition", this.pitchPosition);
        table.put("RollPosition", this.rollPosition);
        table.put("IsCalibrating", this.isCalibrating);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.connected = table.get("Connected", this.connected);
        this.yawPosition = table.get("YawPosition", this.yawPosition);
        this.yawVelocityRadPerSec = table.get("YawVelocityRadPerSec", this.yawVelocityRadPerSec);
        this.pitchPosition = table.get("PitchPosition", this.pitchPosition);
        this.rollPosition = table.get("RollPosition", this.rollPosition);
        this.isCalibrating = table.get("IsCalibrating", this.isCalibrating);
    }
}
