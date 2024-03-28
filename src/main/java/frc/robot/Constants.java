package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.subsystems.base.SubsystemConfig;
import frc.robot.util.math.GearRatio;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMap;

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
    /*
    public static class Debug {
        public static final boolean PHOTON_TUNING_ENABLED = false;
        public static final boolean SHOOTER_TUNING_ENABLED = false; // FIXME: caused issues with slow shoot.
        public static final boolean INDEX_TUNING_ENABLED = false;
        public static final boolean INTAKE_TUNING_ENABLED = false;
        public static final boolean CLIMBER_TUNING_ENABLED = false;
        public static final boolean TRAP_ARM_TUNING_ENABLED = false;
        public static final boolean PHOTON_ENABLED = false;
        public static final boolean SWERVE_TUNING_ENABLED = true;
    }
     */

    public static class Systems {
        public static final SubsystemConfig SHOOT_CAMERA = new SubsystemConfig(
                "ShooterCamera",
                true,
                true
        );
        public static final SubsystemConfig SHOOTER = new SubsystemConfig(
                "Shooter",
                true,
                false
        );
        public static final SubsystemConfig INDEX = new SubsystemConfig(
                "Index",
                true,
                false
        );
        public static final SubsystemConfig INTAKE = new SubsystemConfig(
                "Intake",
                false,
                false
        );
        public static final SubsystemConfig FINGER = new SubsystemConfig(
                "Finger",
                true,
                false
        );
        public static final SubsystemConfig CLIMBER = new SubsystemConfig(
                "Climber",
                false,
                false
        );
        public static final SubsystemConfig SWERVE = new SubsystemConfig(
                "Swerve",
                true,
                false
        );
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
        public static final double SHOOT_SPEED = 1;
        public static final double SHOOT_IDLE_SPEED = 0;
        public static final double SLOW_SHOOT_SPEED = 0.07;

        public static final PIDConstants SHOOT_PID = new PIDConstants(0.1, 0, 0);
    }

    /** This {@link Indexer} class represents all values regarding the {@link Robot}'s index mechanism. */
    public static class Indexer {
        public static final int INDEX_LEFT_MOTOR_ID = 11;
        public static final int INDEX_RIGHT_MOTOR_ID = 15;
        public static final double INDEX_SPEED = 0.6;
        public static final double SLOW_INDEX_SPEED = 0.08;
    }

    /** This {@link Intake} class represents all values regarding the {@link Robot}'s in-taking mechanism. */
    public static class Intake {
        public static final double INTAKE_SPEED = 0.4;
        public static final double SLOW_INTAKE_SPEED = 0.1;
        public static final int INTAKE_MOTOR_ID = 12;
        public static final int INTAKE_SENSOR_PORT = 0;
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

    public static class TrapFinger {
        public static final int ARM_EXTENSION_MOTOR_ID = 18;
        public static final double ARM_MAX_ROTATION = 12;
        public static final GearRatio ARM_GEAR_RATIO = GearRatio.from(12, 1);
        public static final PIDConstants ARM_EXTENSION_PID = new PIDConstants(0.015, 0, 0);
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
        public static final String SHOOT_CAMERA_NAME = "Microsoft_LifeCam_HD-3000";

        public static final Transform3d BACK_CAMERA_TRANSFORM = new Transform3d(
                new Translation3d(0, 0, 0.30),
                new Rotation3d(0, 0, 0)
        );
    }

    public static class Presets {
        public static final PresetMap<Double> TRAP_ARM_PRESETS = new PresetMap<>("Trap Arm", true);

        static {
            TRAP_ARM_PRESETS.put("Zero", 0.0);
            TRAP_ARM_PRESETS.put("One", 3.0);
            TRAP_ARM_PRESETS.put("Two", 12.0);

            // TODO: add real entries!
            Robot.arm.registerExtensionPresets(TRAP_ARM_PRESETS);
            //Robot.arm.registerAnglePresets(TRAP_ARM_ANGLE_PRESETS);
            //Robot.wrist.registerPresets(TRAP_WRIST_PRESETS);
        }

        /** Use this group for interfacing the trap presets!! **/
        public static final PresetGroup TRAP_PRESET_GROUP = new PresetGroup(
                "Trap Group",
                TRAP_ARM_PRESETS
        );
    }

    public static class FrontCamera {

    }



    public static class Chassis {
        public static final double SIDE_LENGTH_METERS = Units.inchesToMeters(30);
        public static final double MAX_SPEED_MPS = 4.602;

        public static final double PHOTON_DRIVE_MAX_SPEED = 0.5;
        public static final double PHOTON_TURN_MAX_SPEED = 0.2;

        public static final double AUTO_DRIVE_MAX_SPEED = 0.5;
        public static final double AUTO_TURN_MAX_SPEED = 0.2;

        public static final PIDConstants PHOTON_DRIVE_PID = new PIDConstants(0.3, 0, 0);
        public static final PIDConstants PHOTON_TURN_PID = new PIDConstants(0.002, 0, 0);

        //public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(0.3, 0, 0);
        //public static final PIDConstants AUTO_TURN_PID = new PIDConstants(0.23, 0, 0);

        public static final PIDConstants AUTO_DRIVE_PID = new PIDConstants(5.25, 0, 0);
        public static final PIDConstants AUTO_TURN_PID = new PIDConstants(4, 0, 0);
    }
}