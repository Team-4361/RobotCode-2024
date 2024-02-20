package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.IDriveMode;
import frc.robot.util.math.GearRatio;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMap;

import java.util.function.Supplier;

import static edu.wpi.first.units.Units.Inches;

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
    public static class Debug {
        public static final boolean PHOTON_TUNING_ENABLED = false;
        public static final boolean SHOOTER_TUNING_ENABLED = false;
        public static final boolean DEBUG_LOGGING_ENABLED = false;
        public static final boolean INDEX_TUNING_ENABLED = true;
        public static final boolean INTAKE_TUNING_ENABLED = true;
        public static final boolean WRIST_TUNING_ENABLED = false;
        public static final boolean CLIMBER_TUNING_ENABLED = true;
        public static final boolean TRAP_ARM_TUNING_ENABLED = false;
        public static final boolean PHOTON_ENABLED = false;
    }


    /** This {@link Shooter} class represents all values regarding the {@link Robot}'s shooting mechanism. */
    public static class Shooter {
        public static final int SHOOT_LEFT_MOTOR_ID = 10;
        public static final int SHOOT_RIGHT_MOTOR_ID = 16;
        public static final double SHOOT_SPEED = 1;
        public static final double SHOOT_KS = 0.1;
        public static final double SHOOT_KV = 0.06;
        public static final double SHOOT_KA = 0;
        public static final double SHOOT_RPM_TOLERANCE = 200;
        public static final long SHOOT_END_DELAY_MS = 2000;
        public static final PIDConstantsAK SHOOT_PID = new PIDConstantsAK(0.05, 0, 0);
    }

    /** This {@link Indexer} class represents all values regarding the {@link Robot}'s index mechanism. */
    public static class Indexer {
        public static final int INDEX_LEFT_MOTOR_ID = 11;
        public static final int INDEX_RIGHT_MOTOR_ID = 15;
        public static final double INDEX_SPEED = -0.4;
    }

    /** This {@link Intake} class represents all values regarding the {@link Robot}'s in-taking mechanism. */
    public static class Intake {
        public static final double INTAKE_SPEED = 0.4;
        public static final double INTAKE_KS = 0;
        public static final double INTAKE_KV = 0;
        public static final double INTAKE_KA = 0;
        public static final int INTAKE_MOTOR_ID = 12;
        public static final boolean INTAKE_INVERTED = false;
        public static final int INTAKE_SENSOR_PORT = 0;
        public static final PIDConstantsAK INTAKE_PID = new PIDConstantsAK(0.02, 0, 0);
    }

    public static class Wrist {
        public static final int WRIST_MAX_US = 2000;
        public static final int WRIST_DEAD_BAND_MAX_US = 1500;
        public static final int WRIST_CENTER_US = 1500;
        public static final int WRIST_DEAD_BAND_MIN_US = 1500;
        public static final int WRIST_MIN_US = 1000;

        public static final int WRIST_MOTOR_ID = 18;
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
        public static final int CLIMBER_LEFT_ID = 13;
        public static final int CLIMBER_RIGHT_ID = 14;
        public static final int CLIMBER_LEFT_DIO = 1;
        public static final int CLIMBER_RIGHT_DIO = 2;
        public static final double CLIMBER_SPEED = 0.5;
        public static final boolean CLIMBER_LEFT_INVERTED = true;
        public static final boolean CLIMBER_RIGHT_INVERTED = false;
    }

    public static class TrapArm {
        // change later?
        public static final int ARM_MAX_US = 2000;
        public static final int ARM_DEAD_BAND_MAX_US = 1500;
        public static final int ARM_CENTER_US = 1500;
        public static final int ARM_DEAD_BAND_MIN_US = 1500;
        public static final int ARM_MIN_US = 1000;

        public static final int ARM_MOTOR_ID = 17;
        public static final int ARM_SERVO_ID = 1;
        public static final double ARM_KS = 0;
        public static final double ARM_KV = 0;
        public static final double ARM_KA = 0;

        public static final double ARM_SERVO_MAX_MM = 50;

        // change later?
        public static final PeakMotorDistance ARM_DISTANCE = new PeakMotorDistance(Inches.of(24), 10);
        public static final PIDConstantsAK ARM_PID = new PIDConstantsAK(0.02, 0, 0);
    }

    public static class Control {
        /** The Left Joystick ID (typically 0) */
        public static final int LEFT_STICK_ID = 0;
        /** The Right Joystick ID (typically 1) */
        public static final int RIGHT_STICK_ID = 1;
        /** The Xbox Controller ID (typically 2) */
        public static final int XBOX_CONTROLLER_ID = 2;

        /** The default dead-zone value to use on Controllers. */
        public static final double DEAD_ZONE = 0.05;

        /** The default Drive Modes to use on the Primary Joysticks (left/right). */
        public static final IDriveMode[] DRIVE_MODES = new IDriveMode[]{
                DriveMode.SMOOTH_MAP,
                DriveMode.LINEAR_MAP,
                DriveMode.SLOW_MODE
        };
    }

    public static class VisionTracking {
        /** The maximum duration a Camera must have no {@link Pose2d} for it to register. */
        public static final double CAMERA_POSE_BUFFER_MILLIS = 500;
        public static final Transform3d CAMERA_OFFSET = new Transform3d(
                new Translation3d(0.3, 0, 0.3),
                new Rotation3d(0, 0, 0)
        );
    }

    public static class AlertConfig {
        public static final Supplier<Long> ALERT_PERIODIC_MS = () -> (long)(DriverStation.isEnabled() ? 1000 : 3000);

        /** The {@link String} used when a Motor is stalled/over-temp. Use '%ID%' to reference the ID. */
        public static final String STRING_MOTOR_OVER_TEMP = "Motor #%ID% stalled/over-temp; output disabled.";

        public static final String STRING_GYRO_CALIBRATING = "Gyroscope calibrating!";
        public static final String STRING_NO_GYRO = "Gyroscope disconnected!";
    }

    public static class Presets {
        public static final PresetMap<Double> TRAP_ARM_PRESETS = new PresetMap<>(
                "Trap Arm",
                true
        );
        public static final PresetMap<Double> TRAP_WRIST_PRESETS = new PresetMap<>(
                "Trap Wrist",
                true
        );
        public static final PresetMap<Double> TRAP_ARM_ANGLE_PRESETS = new PresetMap<>(
                "Trap Angle",
                true
        );

        static {
            TRAP_ARM_PRESETS.put("Zero", 0.0);
            TRAP_WRIST_PRESETS.put("Zero", 0.0);
            TRAP_ARM_ANGLE_PRESETS.put("Zero", 0.0);

            TRAP_ARM_ANGLE_PRESETS.put("One", 20.0);

            // TODO: add real entries!
            Robot.arm.registerExtensionPresets(TRAP_ARM_PRESETS);
            Robot.arm.registerAnglePresets(TRAP_ARM_ANGLE_PRESETS);
            Robot.wrist.registerPresets(TRAP_WRIST_PRESETS);
        }

        /** Use this group for interfacing the trap presets!! **/
        public static final PresetGroup TRAP_PRESET_GROUP = new PresetGroup(
                "Trap Group",
                TRAP_ARM_PRESETS,
                TRAP_WRIST_PRESETS,
                TRAP_ARM_ANGLE_PRESETS
        );
    }

    public static class Chassis {
        // UPDATE: The configuration is now updated with YAGSL using the 'src/main/deploy/swerve' directory.
        public static final double SIDE_LENGTH_METERS = Units.inchesToMeters(30);
        public static final double MAX_SPEED_MPS = 12.5;
        public static final double PHOTON_DRIVE_MAX_SPEED = 0.5;

        public static final PIDConstantsAK PHOTON_DRIVE_PID = new PIDConstantsAK(0.01, 0, 0);
        public static final PIDConstantsAK PHOTON_TURN_PID = new PIDConstantsAK(0.01, 0, 0);
    }
}
