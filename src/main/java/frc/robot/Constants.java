package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.IDriveMode;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.swerve.GyroIONavX1;
import frc.robot.util.swerve.SwerveModuleIOCAN;
import frc.robot.util.swerve.config.ChassisSettings;
import frc.robot.util.swerve.config.Mk4Chassis;
import frc.robot.util.swerve.SwerveModule;

import java.util.function.Supplier;

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

    public enum OperationMode { REAL, REPLAY, SIM}

    public static class Control {
        /** The Left Joystick ID (typically 0) */
        public static final int LEFT_STICK_ID = 0;
        /** The Right Joystick ID (typically 1) */
        public static final int RIGHT_STICK_ID = 1;
        /** The Xbox Controller ID (typically 2) */
        public static final int XBOX_CONTROLLER_ID = 2;

        public static final OperationMode OP_MODE = OperationMode.REAL;

        /** The default deadband value to use on Controllers. */
        public static final double DEADBAND = 0.05;

        /** The default Drive Modes to use on the Primary Joysticks (left/right). */
        public static final IDriveMode[] DRIVE_MODES = new IDriveMode[]{
                DriveMode.SMOOTH_MAP,
                DriveMode.LINEAR_MAP,
                DriveMode.SLOW_MODE
        };

        public static final boolean SWERVE_TUNING_ENABLED = false;
        public static final boolean PHOTON_TUNING_ENABLED = true;
        public static final boolean MOTOR_BURN_FLASH = false;

        public static final DashTunableNumber PHOTON_TURN_MAX_SPEED = new DashTunableNumber("Photon Turn Speed", 0.2, false);
        public static final DashTunableNumber PHOTON_DISTANCE = new DashTunableNumber("Photon Distance", 1, false);
        public static final DashTunableNumber PHOTON_DRIVE_MAX_SPEED = new DashTunableNumber("Photon Max Speed", 0.5, false);
        public static final boolean DEBUG_ENABLED = false;
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
        public static final double MAX_ANGULAR_MPS = CHASSIS_MODE.getMaxSpeed() / CHASSIS_BASE_RADIUS;

        public static final GyroIONavX1 GYRO_MODULE = new GyroIONavX1(SPI.Port.kMXP);

        public static final SwerveModule FL_MODULE = new SwerveModule(
                "FL",
                new SwerveModuleIOCAN(
                        CHASSIS_MODE.getFLDriveID(),
                        CHASSIS_MODE.getFLTurnID(),
                        CHASSIS_MODE.getFLEncoderID(),
                        CHASSIS_MODE.getFLOffsetRad()
                ),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );

        public static final SwerveModule FR_MODULE = new SwerveModule(
                "FR",
                new SwerveModuleIOCAN(
                        CHASSIS_MODE.getFRDriveID(),
                        CHASSIS_MODE.getFRTurnID(),
                        CHASSIS_MODE.getFREncoderID(),
                        CHASSIS_MODE.getFROffset()
                ),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );

        public static final SwerveModule BL_MODULE = new SwerveModule(
                "BL",
                new SwerveModuleIOCAN(
                        CHASSIS_MODE.getBLDriveID(),
                        CHASSIS_MODE.getBLTurnID(),
                        CHASSIS_MODE.getBLEncoderID(),
                        CHASSIS_MODE.getBLOffset()
                ),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );

        public static final SwerveModule BR_MODULE = new SwerveModule(
                "BR",
                new SwerveModuleIOCAN(
                        CHASSIS_MODE.getBRDriveID(),
                        CHASSIS_MODE.getBRTurnID(),
                        CHASSIS_MODE.getBREncoderID(),
                        CHASSIS_MODE.getBROffset()
                ),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );
    }
}
