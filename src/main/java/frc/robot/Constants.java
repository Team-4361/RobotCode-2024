package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.util.math.GearRatio;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMap;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.wpilibj.PowerDistribution.ModuleType.kRev;

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
        public static final boolean INDEX_TUNING_ENABLED = false;
        public static final boolean INTAKE_TUNING_ENABLED = false;
        public static final boolean CLIMBER_TUNING_ENABLED = false;
        public static final boolean TRAP_ARM_TUNING_ENABLED = false;
        public static final boolean TRAP_WRIST_TUNING_ENABLED = false;
        public static final boolean PHOTON_ENABLED = true;
        public static final boolean SWERVE_TUNING_ENABLED = false;
    }

    public static class Power {
        public static final ModuleType POWER_MODULE_TYPE = kRev;
        public static final int POWER_CAN_ID = 34;
    }


    /** This {@link Shooter} class represents all values regarding the {@link Robot}'s shooting mechanism. */
    public static class Shooter {
        public static final int SHOOT_LEFT_MOTOR_ID = 10;
        public static final int SHOOT_RIGHT_MOTOR_ID = 16;
        public static final long SHOOT_END_DELAY_MS = 750;
        public static final double SHOOT_SPEED = 0.9;
        public static final double SLOW_SHOOT_SPEED = 0.1;
    }

    /** This {@link Indexer} class represents all values regarding the {@link Robot}'s index mechanism. */
    public static class Indexer {
        public static final int INDEX_LEFT_MOTOR_ID = 11;
        public static final int INDEX_RIGHT_MOTOR_ID = 15;
        public static final double INDEX_SPEED = 0.4;
        public static final double SLOW_INDEX_SPEED = 0.1;
    }

    /** This {@link Intake} class represents all values regarding the {@link Robot}'s in-taking mechanism. */
    public static class Intake {
        public static final double INTAKE_SPEED = 0.4;
        public static final double SLOW_INTAKE_SPEED = 0.1;
        public static final int INTAKE_MOTOR_ID = 12;
        public static final int INTAKE_SENSOR_PORT = 0;
    }

    public static class Wrist {
        public static final int WRIST_MAX_US = 2000;
        public static final int WRIST_DEAD_BAND_MAX_US = 1500;
        public static final int WRIST_CENTER_US = 1500;
        public static final int WRIST_DEAD_BAND_MIN_US = 1500;
        public static final int WRIST_MIN_US = 1000;
        public static final int WRIST_MOTOR_ID = 18;
        public static final int WRIST_SERVO_ID = 0;
        public static final int WRIST_SERVO_MIN_MM = 0;

        public static final double WRIST_KS = 0;
        public static final double WRIST_KV = 0;
        public static final double WRIST_KA = 0;
        public static final double WRIST_SERVO_MAX_MM = 50;

        public static final boolean WRIST_INVERTED = false;

        public static final GearRatio WRIST_TURN_RATIO = GearRatio.from(63, 1);
        public static final PIDConstants WRIST_PID = new PIDConstants(0.02, 0, 0);
    }

    public static class Climber {
        public static final int CLIMBER_LEFT_ID = 13;
        public static final int CLIMBER_RIGHT_ID = 14;
        public static final int CLIMBER_LEFT_DIO = 1;
        public static final int CLIMBER_RIGHT_DIO = 2;
        public static final double CLIMBER_SPEED = 1;
        public static final boolean CLIMBER_LEFT_INVERTED = true;
        public static final boolean CLIMBER_RIGHT_INVERTED = false;
        public static final long CLIMBER_SENSOR_DELAY_MS = 150;
    }

    public static class TrapArm {
        public static final int ARM_MAX_US = 2000;
        public static final int ARM_DEAD_BAND_MAX_US = 1500;
        public static final int ARM_CENTER_US = 1500;
        public static final int ARM_DEAD_BAND_MIN_US = 1500;
        public static final int ARM_MIN_US = 1000;
        public static final int ARM_SERVO_MIN_MM = 3;

        public static final int ARM_MOTOR_ID = 17;
        public static final int ARM_SERVO_ID = 1;
        public static final double ARM_KS = 0;
        public static final double ARM_KV = 0;
        public static final double ARM_KA = 0;

        public static final double ARM_SERVO_MAX_MM = 50;
        public static final PeakMotorDistance ARM_DISTANCE = new PeakMotorDistance(Inches.of(24), 425.4);
        public static final PIDConstants ARM_PID = new PIDConstants(0.02, 0, 0);
    }

    public static class Control {
        /** The Left Joystick ID (typically 0) */
        public static final int LEFT_STICK_ID = 0;
        /** The Right Joystick ID (typically 1) */
        public static final int RIGHT_STICK_ID = 1;
        /** The Xbox Controller ID (typically 2) */
        public static final int XBOX_CONTROLLER_ID = 2;
    }

    public static class ShooterCamera {
        public static final String SHOOT_CAMERA_NAME = "Shooter Camera";
        public static final double SHOOT_CAMERA_HEIGHT_METERS = Units.inchesToMeters(27);
        public static final double SHOOT_CAMERA_PITCH_DEGREES = 20;
    }

    public static class Presets {
        public static final PresetMap<Double> TRAP_ARM_PRESETS = new PresetMap<>("Trap Arm", true);
        public static final PresetMap<Double> TRAP_WRIST_PRESETS = new PresetMap<>("Trap Wrist", true);
        public static final PresetMap<Double> TRAP_ARM_ANGLE_PRESETS = new PresetMap<>("Trap Angle", true);

        static {
            TRAP_ARM_PRESETS.put("Zero", 0.0);
            TRAP_WRIST_PRESETS.put("Zero", 0.0);
            TRAP_ARM_ANGLE_PRESETS.put("Zero", 0.0);

            TRAP_WRIST_PRESETS.put("One", 216.0);
            TRAP_ARM_PRESETS.put("One", 24.0);
            TRAP_ARM_ANGLE_PRESETS.put("One", 48.3);

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
        public static final double SIDE_LENGTH_METERS = Units.inchesToMeters(30);
        public static final double MAX_SPEED_MPS = 12.5;

        public static final double PHOTON_DRIVE_MAX_SPEED = 0.5;
        public static final double PHOTON_TURN_MAX_SPEED = 0.2;

        public static final PIDConstants PHOTON_DRIVE_PID = new PIDConstants(0.02, 0, 0);
        public static final PIDConstants PHOTON_TURN_PID = new PIDConstants(0.02, 0, 0);

        public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(0.25, 0, 0);
        public static final PIDConstants AUTO_TURN_PID = new PIDConstants(0.02, 0, 0);
    }
}