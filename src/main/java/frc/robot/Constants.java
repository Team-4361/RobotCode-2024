package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.IDriveMode;
import frc.robot.util.math.GearRatio;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.config.ChassisSettings;
import frc.robot.util.swerve.config.Mk4Chassis;

import java.util.LinkedHashMap;
import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Inches;
import static frc.robot.Constants.LooperConfig.*;
import static frc.robot.Constants.LooperConfig.STRING_DASHBOARD_NAME;

/**
 * This {@link Constants} class is an easy-to-use place for fixed value storage (ex. motor/controller IDs,
 * ratios, etc.)
 * <p></p>
 * Only <b>primitive types</b> and <b>Configuration Objects</b> shall be stored here.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class Constants {
    public enum OperationMode { REAL, REPLAY, SIM }

    public static void runIfNotReplay(Runnable runnable) {
        if (Control.OP_MODE != OperationMode.REPLAY)
            runnable.run();
    }

    public static boolean isReplay() { return Control.OP_MODE == OperationMode.REPLAY; }

    ///////////////////////////////////////////////////////////////////////////////

    public static class Debug {
        public static final boolean SWERVE_TUNING_ENABLED = false;
        public static final boolean PHOTON_TUNING_ENABLED = false;
        public static final boolean SHOOTER_TUNING_ENABLED = false;
        public static final boolean DEBUG_LOGGING_ENABLED = false;
        public static final boolean INDEX_TUNING_ENABLED = false;
        public static final boolean INTAKE_TUNING_ENABLED = false;
        public static final boolean WRIST_TUNING_ENABLED = false;
        public static final boolean CLIMBER_TUNING_ENABLED = true;
    }


    /** This {@link Shooter} class represents all values regarding the {@link Robot}'s shooting mechanism. */
    public static class Shooter {
        public static final int LEFT_SHOOTER_MOTOR_ID = 10;
        public static final int RIGHT_SHOOTER_MOTOR_ID = 11;
        public static final double SHOOT_RPM = 5600;
        public static final double SHOOT_KS = 0.1;
        public static final double SHOOT_KV = 0.03;
        public static final double SHOOT_KA = 0;
        public static final double SHOOT_RPM_TOLERANCE = 100;
        public static final PIDConstantsAK SHOOT_PID = new PIDConstantsAK(0.05, 0, 0);
    }

    /** This {@link Indexer} class represents all values regarding the {@link Robot}'s index mechanism. */
    public static class Indexer {
        public static final int INDEX_LEFT_MOTOR_ID = 12;
        public static final int INDEX_RIGHT_MOTOR_ID = 13;

        public static final double INDEX_KS = 0;
        public static final double INDEX_KV = 0;
        public static final double INDEX_KA = 0;
        public static final int INDEX_SENSOR_PORT = 0;
        public static final double INDEX_SPEED = -0.4;
        public static final PIDConstantsAK INDEX_PID = new PIDConstantsAK(0.01, 0, 0);
    }

    /** This {@link Intake} class represents all values regarding the {@link Robot}'s in-taking mechanism. */
    public static class Intake {
        public static final double INTAKE_RPM = 1500;
        public static final double INTAKE_KS = 0;
        public static final double INTAKE_KV = 0;
        public static final double INTAKE_KA = 0;
        public static final int INTAKE_MOTOR_ID = 14;
        public static final boolean INTAKE_INVERTED = false;
        public static final PIDConstantsAK INTAKE_PID = new PIDConstantsAK(0.02, 0, 0);
    }

    public static class Wrist {
        public static final int WRIST_MAX_US = 2000;
        public static final int WRIST_DEAD_BAND_MAX_US = 1500;
        public static final int WRIST_CENTER_US = 1500;
        public static final int WRIST_DEAD_BAND_MIN_US = 1500;
        public static final int WRIST_MIN_US = 1000;

        public static final int WRIST_MOTOR_ID = 15;
        public static final int WRIST_SERVO_ID = 0;
        public static final double WRIST_KS = 0;
        public static final double WRIST_KV = 0;
        public static final double WRIST_KA = 0;

        public static final boolean WRIST_INVERTED = false;
        public static final double WRIST_SERVO_MAX_MM = 50;

        public static final GearRatio WRIST_TURN_RATIO = GearRatio.from(63, 1);
        public static final PIDConstantsAK WRIST_PID = new PIDConstantsAK(0.02, 0, 0);
    }

    public static class Climber{
        public static final int LEFT_CLIMB_MOTOR_ID = 16;
        public static final int RIGHT_CLIMB_MOTOR_ID = 17;
        public static final double CLIMB_KS = 0;
        public static final double CLIMB_KV = 0;
        public static final double CLIMB_KA = 0;
        public static final PIDConstantsAK CLIMB_PID = new PIDConstantsAK(0.05, 0 , 0);
        public static final PeakMotorDistance MAX_DISTANCE = new PeakMotorDistance(Inches.of(24.5), 100);

        public static final LinkedHashMap<String, Double> CLIMBER_PRESETS = new LinkedHashMap<>();
        static {
            CLIMBER_PRESETS.put("Zero", 0.0);
            CLIMBER_PRESETS.put("Top", 24.5);
        }
    }

    public static class Control {
        /** The Left Joystick ID (typically 0) */
        public static final int LEFT_STICK_ID = 0;
        /** The Right Joystick ID (typically 1) */
        public static final int RIGHT_STICK_ID = 1;
        /** The Xbox Controller ID (typically 2) */
        public static final int XBOX_CONTROLLER_ID = 2;

        public static final OperationMode OP_MODE = (RobotBase.isSimulation()) ? OperationMode.SIM : OperationMode.REAL;

        /** The default dead-zone value to use on Controllers. */
        public static final double DEAD_ZONE = 0.05;

        /** The default Drive Modes to use on the Primary Joysticks (left/right). */
        public static final IDriveMode[] DRIVE_MODES = new IDriveMode[]{
                DriveMode.SMOOTH_MAP,
                DriveMode.LINEAR_MAP,
                DriveMode.SLOW_MODE
        };

        public static final DashTunableNumber PHOTON_TURN_MAX_SPEED = new DashTunableNumber(
                "Photon: Turn Speed",
                0.2,
                false
        );
        public static final DashTunableNumber PHOTON_DISTANCE = new DashTunableNumber(
                "Photon: Distance",
                1,
                false
        );
        public static final DashTunableNumber PHOTON_DRIVE_MAX_SPEED = new DashTunableNumber(
                "Photon: Max Speed",
                0.5,
                false
        );

        static {
            IOManager.initLoop(STRING_PERIODIC_NAME, PERIODIC_INTERVAL);
            IOManager.initLoop(STRING_DASHBOARD_NAME, DASHBOARD_INTERVAL);
            IOManager.initLoop(STRING_ODOMETRY_NAME, ODOMETRY_INTERVAL);

            IOManager.addPeriodicIfExists(STRING_DASHBOARD_NAME, PHOTON_DISTANCE::update);
            IOManager.addPeriodicIfExists(STRING_DASHBOARD_NAME, PHOTON_DRIVE_MAX_SPEED::update);
            IOManager.addPeriodicIfExists(STRING_DASHBOARD_NAME, PHOTON_TURN_MAX_SPEED::update);
        }
    }

    public static class VisionTracking {
        /** The maximum duration a Camera must have no {@link Pose2d} for it to register. */
        public static final double CAMERA_POSE_BUFFER_MILLIS = 500;
      //  public static final AprilTagFieldLayout FIELD_LAYOUT = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
        public static final Transform3d CAMERA_OFFSET = new Transform3d(
                new Translation3d(0.3, 0, 0.3),
                new Rotation3d(0, 0, 0)
        );
    }

    public static class AlertConfig {
        public static final Supplier<Long> ALERT_PERIODIC_MS = () -> (long)(DriverStation.isEnabled() ? 1000 : 3000);
        public static final String STRING_HIGH_PERIODIC_MS = "Slow main Thread loop (>= 25ms)";

        /** The {@link String} used when a Motor is stalled/over-temp. Use '%ID%' to reference the ID. */
        public static final String STRING_MOTOR_OVER_TEMP = "Motor #%ID% stalled/over-temp; output disabled.";

        public static final String STRING_GYRO_CALIBRATING = "Gyroscope calibrating!";
        public static final String STRING_PROGRAMMING_MOTOR = "Programing Motors...";
    }

    /**
     * Holds all {@link Constants} for the {@link IOManager} class. This mainly holds the milliseconds for
     * Normal and Simulation operations.
     */
    public static class LooperConfig {
        /** The <b>default</b> millisecond loop time. Note: this can be overridden per Looper interface. */
        public static final String STRING_PERIODIC_NAME = "PERIODIC";
        public static final long PERIODIC_INTERVAL = 20;

        public static final String STRING_ODOMETRY_NAME = "ODOMETRY";
        public static final long ODOMETRY_INTERVAL = 50;

        public static final String STRING_DASHBOARD_NAME = "DASHBOARD";
        public static final long DASHBOARD_INTERVAL = 250;
    }


    /**
     * This {@link Mk4SDSRatio} enum represents commonly used Gear Reductions on the SDS Mk4.
     * <a href="https://cdn.shopify.com/s/files/1/0065/4308/1590/files/MK4DrivetrainFreeSpeeds_caf3d1d5-2f83-43f5-acd9-15ef4f6cc3cc_1024x1024.png">The link this class was based upon.</a>
     */
    public enum Mk4SDSRatio {
        L1(8.14),
        L2(6.75),
        L3(6.12),
        L4(5.14);

        /** @return The starting ratio (X:1) of this setup. */
        public double getRatio() { return this.ratio; }

        private final double ratio;

        /**
         * Constructs a new {@link Mk4SDSRatio} with the parameter.
         * @param ratio The ratio to use against 1 (X:1)
         */
        Mk4SDSRatio(double ratio) {
            this.ratio = ratio;
        }
    }


    public static class Chassis {
        public static final ChassisSettings CHASSIS_MODE = new Mk4Chassis();
        public static final double CHASSIS_BASE_RADIUS = Math.hypot(
                CHASSIS_MODE.getSideLength() / 2.0,
                CHASSIS_MODE.getSideLength() / 2.0
        );
       // public static final double MAX_ANGULAR_MPS = CHASSIS_MODE.getMaxSpeed() / CHASSIS_BASE_RADIUS;
    }
}
