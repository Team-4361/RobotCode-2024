package frc.robot;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.util.io.IOManager;
import frc.robot.util.joystick.DriveMode;
import frc.robot.util.joystick.IDriveMode;
import frc.robot.util.math.GearRatio;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMap;
import frc.robot.util.preset.PresetMode;
import frc.robot.util.swerve.SwerveModule;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

import java.time.Duration;
import java.util.function.Supplier;

import static java.util.Map.entry;
import static java.util.Map.ofEntries;

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
                DriveMode.LINEAR_MAP
        };
    }

    public static class AlertConfig {
        public static final Supplier<Long> ALERT_PERIODIC_MS = () -> (long)(DriverStation.isEnabled() ? 1000 : 3000);
        public static final String STRING_HIGH_PERIODIC_MS = "Slow main Thread loop (>= 25ms)";

        /** The {@link String} used when a Motor is stalled/over-temp. Use '%ID%' to reference the ID. */
        public static final String STRING_MOTOR_OVER_TEMP = "Motor #%ID% stalled/over-temp; output disabled.";

        public static final String STRING_SWERVE_CONFIG_FAIL = "Swerve Config Failed! DO NOT DRIVE!";
    }

    /**
     * Holds all {@link Constants} for the {@link IOManager} class. This mainly holds the milliseconds for
     * Normal and Simulation operations.
     */
    public static class LooperConfig {
        /** The <b>default</b> millisecond loop time. Note: this can be overridden per Looper interface. */
        public static final String STRING_PERIODIC_NAME = "PERIODIC";
        public static final Duration PERIODIC_INTERVAL = Duration.ofMillis(20);

        public static final String STRING_LOW_PRIORITY_NAME = "LOW-PRIORITY";
        public static final Duration LOW_PRIORITY_INTERVAL = Duration.ofSeconds(3);
    }

    /**
     * This {@link Chassis} class is designed to store values regarding Driving the robot. This includes any offsets
     * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
     * need to be flipped due to the Field Oriented driving system.
     */
    public static class Chassis {
        /** The offset of the Front Right Motor */
        public static final double FR_OFFSET = ((-2.38)+0)+(Math.PI/2) - (2 * Math.PI) + (Math.PI);

        /** The offset of the Front Left Motor */
        public static final double FL_OFFSET = ((9.401)+0.045647)+(Math.PI/2) - (Math.PI / 2);

        /** The offset of the Back Right Motor */
        public static final double BR_OFFSET =  ((-3.345)+0.009)+(Math.PI/2) - (Math.PI / 2) - (2 * Math.PI);

        /** The offset of the Back Left Motor */
        public static final double BL_OFFSET = ((6.12)+0.339057)+(Math.PI/2) - (2 * Math.PI) - (Math.PI / 2);

        /** The length of the side of the {@link Chassis} in <b>meters.</b> */
        public static final double SWERVE_CHASSIS_SIDE_LENGTH = 0.762;

        /** The Motor ID used for the Front Right Drive Motor. */
        public static final int FR_DRIVE_ID = 4;

        /** The Motor ID used for the Front Left Drive Motor. */
        public static final int FL_DRIVE_ID = 2;

        /** The Motor ID used for the Back Right Drive Motor. */
        public static final int BR_DRIVE_ID = 8;

        /** The Motor ID used for the Back Left Drive Motor. */
        public static final int BL_DRIVE_ID = 6;

        /** The Motor ID used for the Front Right Steering Motor. */
        public static final int FR_TURN_ID = 3;

        /** The Motor ID used for the Front Left Steering Motor. */
        public static final int FL_TURN_ID = 1;

        /** The Motor ID used for the Back Right Steering Motor. */
        public static final int BR_TURN_ID = 7;

        /** The Motor ID used for the Back Left Steering Motor. */
        public static final int BL_TURN_ID = 5;

        /** The ID used for the Front Right Absolute Encoder. */
        public static final int FR_DIO_ENCODER_PORT = 1;

        /** The ID used for the Front Left Absolute Encoder. */
        public static final int FL_DIO_ENCODER_PORT = 0;

        /** The ID used for the Back Right Absolute Encoder. */
        public static final int BR_DIO_ENCODER_PORT = 3;

        /** The ID used for the Back Left Absolute Encoder. */
        public static final int BL_DIO_ENCODER_PORT = 2;

        /** The Radius of each of the Swerve Drive Wheels in <b>meters.</b> */
        public static final double SWERVE_WHEEL_RADIUS = 0.0508;

        /** The Circumference of each of the Swerve Drive Wheels in <b>meters.</b> */
        public static final double SWERVE_WHEEL_CIRCUMFERENCE = SWERVE_WHEEL_RADIUS * 2 * Math.PI;

        /** The {@link GearRatio} of the SDS driving wheels. */
        public static final GearRatio DRIVE_GEAR_RATIO = GearRatio.from(6.86, 1);

        /** The {@link GearRatio} of the SDS turning wheels. */
        public static final GearRatio DRIVE_TURN_RATIO = GearRatio.from(12.8, 1);

        /** The maximum speed of the Swerve Drive system in meters per second. */
        public static final double MAX_SPEED_MPS = 12;

        public static final PIDConstants DRIVE_PID_CONFIG = new PIDConstants(0.05, 0, 0);
        public static final PIDConstants TURN_PID_CONFIG = new PIDConstants(5, 0, 0);

        public static final SwerveModule FL_MODULE = new SwerveModule(
                FL_DRIVE_ID,
                FL_TURN_ID,
                FL_DIO_ENCODER_PORT,
                FL_OFFSET,
                DRIVE_PID_CONFIG,
                TURN_PID_CONFIG
        );

        public static final SwerveModule FR_MODULE = new SwerveModule(
                FR_DRIVE_ID,
                FR_TURN_ID,
                FR_DIO_ENCODER_PORT,
                FR_OFFSET,
                DRIVE_PID_CONFIG,
                TURN_PID_CONFIG
        );

        public static final SwerveModule BL_MODULE = new SwerveModule(
                BL_DRIVE_ID,
                BL_TURN_ID,
                BL_DIO_ENCODER_PORT,
                BL_OFFSET,
                DRIVE_PID_CONFIG,
                TURN_PID_CONFIG
        );

        public static final SwerveModule BR_MODULE = new SwerveModule(
                BR_DRIVE_ID,
                BR_TURN_ID,
                BR_DIO_ENCODER_PORT,
                BR_OFFSET,
                DRIVE_PID_CONFIG,
                TURN_PID_CONFIG
        );
    }

    public static class VacuumValues {
        public static final int[] VACUUM_MOTOR_IDS = new int[]{20, 16, 13, 11};
        public static final double VACUUM_PUMP_SPEED = 0.45;
        public static final MotorType VACUUM_MOTOR_TYPE = MotorType.kBrushed;

        public static final int[][] VACUUM_SOLENOIDS = new int[][]{
                new int[]{1, 7}, // PDH 0
                new int[]{3, 4} // PDH 1
        };

        public static final int[] VACUUM_SENSORS = new int[]{0, 1, 2, 3};
        public static final double VACUUM_THRESHOLD = 1;

    }

    public static class ClimberWristValues {
        public static final int WRIST_GEAR_RATIO = 100;
        public static final int WRIST_MOTOR_ID = 22;
    }


    public static class ClimberPresets {
        public static final String ROTATION_NAME = "CLI ROT";
        public static final String EXTENSION_NAME = "CLI EXT";
        public static final String WRIST_NAME = "CLIM WST";

        public static final String ZERO_POSITION_NAME = "ZERO_POSITION_INDEX";
        public static final String HUMAN_STATION_NAME = "HUMAN_STATION_INDEX";
        public static final String FLOOR_CONE_NAME = "FLOOR_CONE_INDEX";
        public static final String MID_CONE_NAME = "MID_CONE_INDEX";
        public static final String HIGH_CONE_NAME = "HIGH_CONE_INDEX";
        public static final String FLOOR_CUBE_NAME = "FLOOR_CUBE_INDEX";
        public static final String MANUAL_STATION_NAME = "MANUAL_STATION_INDEX";
        public static final String GRAB_FLOOR_CUBE_NAME = "GRAB_FLOOR_CUBE_INDEX";

        public static final PresetMap<Double> ROTATION_PRESETS = new PresetMap<>("Arm Rotation",
                ofEntries(
                        entry(ZERO_POSITION_NAME, 0.0),
                        entry(HUMAN_STATION_NAME, -45.0),
                        entry(FLOOR_CONE_NAME, -143.0),
                        entry(MID_CONE_NAME, -57.0),
                        entry(HIGH_CONE_NAME, -59.0),
                        entry(FLOOR_CUBE_NAME, -102.0),
                        entry(MANUAL_STATION_NAME, -66.0),
                        entry(GRAB_FLOOR_CUBE_NAME, -143.0)
                )
        );

        public static final PresetMap<Double> EXTENSION_PRESETS = new PresetMap<>("Arm Extension",
                ofEntries(
                        entry(ZERO_POSITION_NAME, 0.0),
                        entry(HUMAN_STATION_NAME, 17.365),
                        entry(FLOOR_CONE_NAME, 6.0),
                        entry(MID_CONE_NAME, 9.755),
                        entry(HIGH_CONE_NAME, 49.753),
                        entry(FLOOR_CUBE_NAME, 12.0),
                        entry(MANUAL_STATION_NAME, 0.0),
                        entry(GRAB_FLOOR_CUBE_NAME, 12.0)
                )
        );

        public static final PresetMap<Double> WRIST_PRESETS = new PresetMap<>("Wrist Rotation",
                ofEntries(
                        entry(ZERO_POSITION_NAME, 0.0),
                        entry(HUMAN_STATION_NAME, -38.0),
                        entry(FLOOR_CONE_NAME, 56.0),
                        entry(MID_CONE_NAME, 10.0),
                        entry(HIGH_CONE_NAME, 0.685),
                        entry(FLOOR_CUBE_NAME, -75.0),
                        entry(MANUAL_STATION_NAME, 21.0),
                        entry(GRAB_FLOOR_CUBE_NAME, -75.0)
                )
        );

        public static final PresetGroup CLIMBER_PRESET_GROUP = new PresetGroup(
                "Climber Presets",
                PresetMode.PARALLEL,
                ROTATION_PRESETS,
                EXTENSION_PRESETS,
                WRIST_PRESETS
        );
    }
}
