package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.IDriveMode;
import frc.robot.util.math.GearRatio;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.DashTunablePID;
import frc.robot.util.pid.FRCDistanceMechanism;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMap;
import frc.robot.util.preset.PresetMode;
import frc.robot.util.swerve.SwerveModule;
import org.photonvision.PhotonCamera;

import java.util.function.Supplier;

import static edu.wpi.first.units.BaseUnits.Distance;
import static edu.wpi.first.units.Units.Inches;
import static frc.robot.util.preset.PresetMode.PARALLEL;

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
    public static class Control {
        /** The Left Joystick ID (typically 0) */
        public static final int LEFT_STICK_ID = 0;
        /** The Right Joystick ID (typically 1) */
        public static final int RIGHT_STICK_ID = 1;
        /** The Xbox Controller ID (typically 2) */
        public static final int XBOX_CONTROLLER_ID = 2;

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
     * This {@link ChassisSettings} interface is designed to store values regarding Driving the robot. This includes any offsets
     * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
     * need to be flipped due to the Field Oriented driving system.
     */
    public interface ChassisSettings {
        /** @return The front-left offset. */
        double getFLOffset();

        /** @return The front-right offset. */
        double getFROffset();

        /** @return The back-left offset. */
        double getBLOffset();

        /** @return The back-right offset. */
        double getBROffset();

        /** @return The {@link Robot} side length in <b>meters.</b> */
        double getSideLength();

        /** @return The front-left drive ID. */
        int getFLDriveID();

        /** @return The front-right drive ID. */
        int getFRDriveID();

        /** @return The back-left drive ID. */
        int getBLDriveID();

        /** @return The back-right drive ID. */
        int getBRDriveID();

        /** @return The front-left turn ID. */
        int getFLTurnID();

        /** @return The front-right turn ID. */
        int getFRTurnID();

        /** @return The back-left turn ID. */
        int getBLTurnID();

        /** @return The back-right turn ID. */
        int getBRTurnID();
        
        /** @return The encoder ID of the front-left module. */
        int getFLEncoderID();

        /** @return The encoder ID of the front-right module. */
        int getFREncoderID();

        /** @return The encoder ID of the back-left module. */
        int getBLEncoderID();

        /** @return The encoder ID of the back-right module. */
        int getBREncoderID();

        /** @return The {@link Robot} wheel radius in <b>meters.</b> */
        double getWheelRadius();

        /** @return The {@link GearRatio} used for driving. */
        GearRatio getDriveRatio();

        /** @return The maximum attainable speed of the {@link Robot} in m/s. */
        double getMaxSpeed();

        /** @return The {@link PIDConstants} used for closed-loop control. */
        PIDConstants getDrivePID();

        /** @return The {@link PIDConstants} used for turning. */
        PIDConstants getTurnPID();

        /** @return The {@link PIDConstants} used for {@link PathPlannerAuto} closed-loop control. */
        PIDConstants getAutoDrivePID();

        /** @return The {@link PIDConstants} used for {@link PathPlannerAuto} turning. */
        PIDConstants getAutoTurnPID();

        /** @return The {@link PIDConstants} used for {@link PhotonCamera} closed-loop control. */
        PIDConstants getPhotonDrivePID();

        /** @return The {@link PIDConstants} used for {@link PhotonCamera} turning. */
        PIDConstants getPhotonTurnPID();
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

        public double getRatio() { return this.ratio; }

        private final double ratio;

        Mk4SDSRatio(double ratio) {
            this.ratio = ratio;
        }
    }

    public static class Mk4Chassis implements ChassisSettings {
        /** @return The front-left offset. */
        @Override public double getFLOffset() { return 0; }

        /** @return The front-right offset. */
        @Override public double getFROffset() { return 0; }

        /** @return The back-left offset. */
        @Override public double getBLOffset() { return 0; }

        /** @return The back-right offset. */
        @Override public double getBROffset() { return 0; }

        /** @return The Robot side length in meters. */
        @Override public double getSideLength() { return 0.867; } // 34 inches (30 in + bumper estimate)

        /** @return The front-left drive ID. */
        @Override public int getFLDriveID() { return 2; }

        /** @return The front-right drive ID. */
        @Override public int getFRDriveID() { return 4; }

        /** @return The back-left drive ID. */
        @Override public int getBLDriveID() { return 6; }

        /** @return The back-right drive ID. */
        @Override public int getBRDriveID() { return 8; }

        /** @return The front-left turn ID. */
        @Override public int getFLTurnID() { return 1; }

        /** @return The front-right turn ID. */
        @Override public int getFRTurnID() { return 3; }

        /** @return The back-left turn ID. */
        @Override public int getBLTurnID() { return 5; }

        /** @return The back-right turn ID. */
        @Override public int getBRTurnID() { return 7; }

        /** @return The encoder ID of the front-left module. */
        @Override public int getFLEncoderID() { return 0; }

        /** @return The encoder ID of the front-right module. */
        @Override public int getFREncoderID() { return 1; }

        /** @return The encoder ID of the back-left module. */
        @Override public int getBLEncoderID() { return 2; }

        /** @return The encoder ID of the back-right module. */
        @Override public int getBREncoderID() { return 3; }

        /** @return The Robot wheel radius in meters. */
        @Override public double getWheelRadius() { return 0.0508; }

        /** @return The GearRatio used for driving. */
        @Override public GearRatio getDriveRatio() { return GearRatio.from(Mk4SDSRatio.L2.getRatio(), 1); }

        /** @return The maximum attainable speed of the Robot in m/s. */
        @Override public double getMaxSpeed() { return 12.5; }

        /** @return The PIDConstants used for closed-loop control. */
        @Override public PIDConstants getDrivePID() { return new PIDConstants(2e-4, 0, 0); }

        /** @return The PIDConstants used for turning. */
        @Override public PIDConstants getTurnPID() { return new PIDConstants(0.5, 0, 0); }

        /** @return The PIDConstants used for PathPlannerAuto closed-loop control. */
        @Override public PIDConstants getAutoDrivePID() { return new PIDConstants(5, 0, 0); }

        /** @return The PIDConstants used for PathPlannerAuto turning. */
        @Override public PIDConstants getAutoTurnPID() { return new PIDConstants(1,0,0); }

        /** @return The PIDConstants used for PhotonCamera closed-loop control. */
        @Override public PIDConstants getPhotonDrivePID() { return new PIDConstants(0.5, 0, 0); }

        /** @return The PIDConstants used for PhotonCamera turning. */
        @Override public PIDConstants getPhotonTurnPID() { return new PIDConstants(0.1, 0, 0); }
    }

    public static class Mk3Chassis implements ChassisSettings {
        /** @return The front-left offset. */
        @Override public double getFLOffset() { return  ((9.401)+0.045647)+(Math.PI/2) - (Math.PI / 2); }

        /** @return The front-right offset. */
        @Override public double getFROffset() { return  ((-2.38)+0)+(Math.PI/2) - (2 * Math.PI) + (Math.PI); }

        /** @return The back-left offset. */
        @Override public double getBLOffset() { return ((6.12)+0.339057)+(Math.PI/2) - (2 * Math.PI) - (Math.PI / 2); }

        /** @return The back-right offset. */
        @Override public double getBROffset() { return ((-3.345)+0.009)+(Math.PI/2) - (Math.PI / 2) - (2 * Math.PI); }

        /** @return The Robot side length in meters. */
        @Override public double getSideLength() { return 0.762; }

        /** @return The front-left drive ID. */
        @Override public int getFLDriveID() { return 2; }

        /** @return The front-right drive ID. */
        @Override public int getFRDriveID() { return 4; }

        /** @return The back-left drive ID. */
        @Override public int getBLDriveID() { return 6; }

        /** @return The back-right drive ID. */
        @Override public int getBRDriveID() { return 8; }

        /** @return The front-left turn ID. */
        @Override public int getFLTurnID() { return 1; }

        /** @return The front-right turn ID. */
        @Override public int getFRTurnID() { return 3; }

        /** @return The back-left turn ID. */
        @Override public int getBLTurnID() { return 5; }

        /** @return The back-right turn ID. */
        @Override public int getBRTurnID() { return 7; }

        /** @return The encoder ID of the front-left module. */
        @Override public int getFLEncoderID() { return 0; }

        /** @return The encoder ID of the front-right module. */
        @Override public int getFREncoderID() { return 1; }

        /** @return The encoder ID of the back-left module. */
        @Override public int getBLEncoderID() { return 2; }

        /** @return The encoder ID of the back-right module. */
        @Override public int getBREncoderID() { return 3; }

        /** @return The Robot wheel radius in meters. */
        @Override public double getWheelRadius() { return 0.0508; }

        /** @return The GearRatio used for driving. */
        @Override public GearRatio getDriveRatio() { return GearRatio.from(6.86, 1); }

        /** @return The maximum attainable speed of the Robot in m/s. */
        @Override public double getMaxSpeed() { return 12.5; }

        /** @return The PIDConstants used for closed-loop control. */
        @Override public PIDConstants getDrivePID() { return new PIDConstants(2e-4, 0, 0); }

        /** @return The PIDConstants used for turning. */
        @Override public PIDConstants getTurnPID() { return new PIDConstants(0.5, 0, 0); }

        /** @return The PIDConstants used for PathPlannerAuto closed-loop control. */
        @Override public PIDConstants getAutoDrivePID() { return new PIDConstants(5, 0, 0); }

        /** @return The PIDConstants used for PathPlannerAuto turning. */
        @Override public PIDConstants getAutoTurnPID() { return new PIDConstants(1,0,0); }

        /** @return The PIDConstants used for PhotonCamera closed-loop control. */
        @Override public PIDConstants getPhotonDrivePID() { return new PIDConstants(0.5, 0, 0); }

        /** @return The PIDConstants used for PhotonCamera turning. */
        @Override public PIDConstants getPhotonTurnPID() { return new PIDConstants(0.1, 0, 0); }
    }


    public static class Chassis {
        public static final ChassisSettings CHASSIS_MODE = new Mk3Chassis();

        public static final SwerveModule FL_MODULE = new SwerveModule(
                "FL",
                CHASSIS_MODE.getFLDriveID(),
                CHASSIS_MODE.getFLTurnID(),
                CHASSIS_MODE.getFLEncoderID(),
                CHASSIS_MODE.getFLOffset(),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );

        public static final SwerveModule FR_MODULE = new SwerveModule(
                "FR",
                CHASSIS_MODE.getFRDriveID(),
                CHASSIS_MODE.getFRTurnID(),
                CHASSIS_MODE.getFREncoderID(),
                CHASSIS_MODE.getFROffset(),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );

        public static final SwerveModule BL_MODULE = new SwerveModule(
                "BL",
                CHASSIS_MODE.getBLDriveID(),
                CHASSIS_MODE.getBLTurnID(),
                CHASSIS_MODE.getBLEncoderID(),
                CHASSIS_MODE.getBLOffset(),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );

        public static final SwerveModule BR_MODULE = new SwerveModule(
                "BR",
                CHASSIS_MODE.getBRDriveID(),
                CHASSIS_MODE.getBRTurnID(),
                CHASSIS_MODE.getBREncoderID(),
                CHASSIS_MODE.getBROffset(),
                CHASSIS_MODE.getDrivePID(),
                CHASSIS_MODE.getTurnPID()
        );
    }
}
