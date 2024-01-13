package frc.robot.util.math;

import java.math.BigDecimal;
import java.math.RoundingMode;
import java.time.Duration;
import java.util.List;
import java.util.Random;

/**
 * This {@link ExtendedMath} class provides features which are not integrated
 * into regular Java/WPILib Math classes.
 *
 * @author Eric Gold
 * @since 0.0.0
 */
public class ExtendedMath {
    private static final Random rand = new Random();

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

        return ExtendedMath.inTolerance(eMillis, aMillis, tMillis);
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
}
