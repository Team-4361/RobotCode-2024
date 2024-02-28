package frc.robot.util.math;

import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.time.Duration;
import java.util.List;
import java.util.Random;

/**
 * This {@link GlobalUtils} class provides features which are not integrated
 * into regular Java/WPILib Math classes.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class GlobalUtils {
    public static final Random rand = new Random();

    public static double getDualSpeed(double negativeAxis, double positiveAxis) {
        negativeAxis = deadband(Math.abs(negativeAxis));
        positiveAxis = deadband(Math.abs(positiveAxis));
        if (negativeAxis > 0) { return -negativeAxis; }
        if (positiveAxis > 0) { return positiveAxis; }
        return 0;
    }

    /**
     * @param val Value that is rounded to the second decimal place.
     * @return A rounded {@link Double} to the second decimal place.
     */
    public static double round(double val) {
        return Math.round(val * 100.0) / 100.0;
    }
    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The value to be expected.
     * @param actual The actual value.
     * @param tolerance The maximum error or tolerance that the value can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    public static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    public static boolean inToleranceNotZero(double expected, double actual, double tolerance) {
        return !inTolerance(0, actual, tolerance) && inTolerance(expected, actual, tolerance);
    }

    public static boolean inRange(double value, double min, double max) {
        return value >= min && value <= max;
    }

    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The value to be expected.
     * @param actual The actual value.
     * @param tolerance The maximum error or tolerance that the value can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    public static boolean inTolerance(long expected, long actual, long tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The expected {@link Duration}.
     * @param actual The actual {@link Duration}.
     * @param tolerance The maximum error or tolerance that the {@link Duration} can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    public static boolean inTolerance(Duration expected, Duration actual, Duration tolerance) {
        long eMillis = expected.toMillis();
        long aMillis = actual.toMillis();
        long tMillis = tolerance.toMillis();

        return GlobalUtils.inTolerance(eMillis, aMillis, tMillis);
    }

    /**
     * Converts a millisecond time into H(H):MM:SS
     * @param millis The {@link Long} millisecond time to input.
     * @return The formatted {@link String} time.
     */
    public static String formatTime(long millis) {
        long seconds = millis / 1000;
        long minutes = seconds / 60;
        long hours = minutes / 60;
        seconds %= 60;
        minutes %= 60;

        // Extract last 3 digits of milliseconds
        long milliseconds = millis % 1000;

        return String.format("%d:%02d:%02d.%03d", hours, minutes, seconds, milliseconds);
    }

    /**
     * Converts a millisecond time into H(H):MM:SS
     * @param duration The {@link Duration} to input.
     * @return The formatted {@link String} time.
     */
    public static String formatTime(Duration duration) {
        return formatTime(duration.toMillis());
    }

    /**
     * Calculates the average of a {@link List} of numbers.
     * @param values The {@link List} to use.
     * @return The calculated average.
     */
    public static double average(List<Long> values) {
        if (values == null || values.isEmpty()) {
            throw new IllegalArgumentException("List is null or empty");
        }
        long sum = 0;
        for (long number : values) {
            sum += number;
        }
        return (double) sum/values.size();
    }

    public static double deadband(double value) { return deadband(value, 0.05); }
    public static double deadband(double value, double min) { return (Math.abs(value) <= min) ? 0 : value; }

    /**
     * Calculates the average of a {@link List} of numbers.
     * @param values The {@link List} to use.
     * @return The calculated average.
     */
    public static double averageDouble(List<Double> values) {
        if (values == null || values.isEmpty()) {
            throw new IllegalArgumentException("List is null or empty");
        }
        double sum = 0;
        for (double number : values) {
            sum += number;
        }
        return sum/values.size();
    }

    //https://github.com/SuperiorRoboworksTeam857/2024Crescendo/blob/main/src/main/java/frc/robot/subsystems/SwerveModule.java

    /**
     * Minimize the change in heading the desired swerve module state would require by potentially
     * reversing the direction the wheel spins. Customized from WPILib's version to include placing in
     * appropriate scope for CTRE and REV onboard control as both controllers as of writing don't have
     * support for continuous input.
     *
     * @param desiredState The desired state.
     * @param currentAngle The current module angle.
     */
    public static SwerveModuleState optimize(SwerveModuleState desiredState, Rotation2d currentAngle) {
        double targetAngle =
                placeInAppropriate0To360Scope(currentAngle.getDegrees(), desiredState.angle.getDegrees());
        double targetSpeed = desiredState.speedMetersPerSecond;
        double delta = targetAngle - currentAngle.getDegrees();
        if (Math.abs(delta) > 90) {
            targetSpeed = -targetSpeed;
            targetAngle += delta > 90 ? -180 : 180;
        }
        return new SwerveModuleState(targetSpeed, Rotation2d.fromDegrees(targetAngle));
    }

    /**
     * @param scopeReference Current Angle
     * @param newAngle Target Angle
     * @return Closest angle within scope
     */
    private static double placeInAppropriate0To360Scope(double scopeReference, double newAngle) {
        double lowerBound;
        double upperBound;
        double lowerOffset = scopeReference % 360;
        if (lowerOffset >= 0) {
            lowerBound = scopeReference - lowerOffset;
            upperBound = scopeReference + (360 - lowerOffset);
        } else {
            upperBound = scopeReference - lowerOffset;
            lowerBound = scopeReference - (360 + lowerOffset);
        }
        while (newAngle < lowerBound) { newAngle += 360; }
        while (newAngle > upperBound) { newAngle -= 360; }
        if (newAngle - scopeReference > 180) { newAngle -= 360; }
        else if (newAngle - scopeReference < -180) { newAngle += 360; }
        return newAngle;
    }
}
