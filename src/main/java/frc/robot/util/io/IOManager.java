package frc.robot.util.io;

import com.revrobotics.REVLibError;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import java.lang.management.ManagementFactory;
import java.lang.management.RuntimeMXBean;
import java.util.*;
import java.util.function.Function;
import java.util.function.Supplier;

import static frc.robot.Constants.AlertConfig.ALERT_PERIODIC_MS;
import static frc.robot.Constants.AlertConfig.STRING_HIGH_PERIODIC_MS;

/**
 * This {@link IOManager} class is designed to control ALL {@link Looper} and {@link Alert} instances. Since
 * the original two managers were lightweight and operated in a similar fashion, the decision was made to combine them.
 *
 * @author Eric Gold
 * @since 0.0.1
 */
public class IOManager {
    private static final List<Looper> LOOPS = new ArrayList<>();
    private static final List<Alert> ALERTS = new ArrayList<>();
    private static final RuntimeMXBean MX_BEAN = ManagementFactory.getRuntimeMXBean();

    private static long lastLoopUpdate = 0;
    private static long lastAlertUpdate = 0;

    private static final long testModeDisable = System.currentTimeMillis() + 5000;

    public static List<Looper> getLoops() { return LOOPS; }
    public static List<Alert> getAlerts() { return ALERTS; }

    static {
        getAlert(STRING_HIGH_PERIODIC_MS, AlertType.WARNING)
                .setCondition(() -> System.currentTimeMillis() - lastLoopUpdate >= 25)
                .setEnableDelay(2000)
                .setDisableDelay(2000)
                .setPersistence(false)
                .setOneUse(false);
    }

    public static Looper getLoop(String loopName) {
        Looper loop = LOOPS.stream()
                .filter(o -> o.getName().equals(loopName))
                .findFirst()
                .orElseGet(() -> new Looper(loopName));
        if (!LOOPS.contains(loop))
            LOOPS.add(loop);
        return loop;
    }

    public static Alert getAlert(String alertName, AlertType type) {
        Alert alert = ALERTS.stream()
                .filter(o -> o.getName().equalsIgnoreCase(alertName) && o.getType() == type)
                .findFirst()
                .orElseGet(() -> new Alert(alertName, type));
        if (!ALERTS.contains(alert))
            ALERTS.add(alert);
        return alert;
    }

    /**
     * Generates the log header for a log message.
     *
     * @param sender The originating {@link Object}.
     * @param type   The type of log message.
     * @return The log header.
     */
    private static String getLogHeader(Object sender, String type) {
        String name = " ";
        try {
            if (sender != null)
                name = " " + sender.getClass().getName() + " ";
        } catch (Exception ignored) {
        }
        return "[" + type + name + formatTime(MX_BEAN.getUptime()) + "] ";
    }

    /**
     * Broadcasts a DEBUG message to the RIOLOG.
     *
     * @param sender The originating {@link Object}.
     * @param text   The {@link String} to broadcast.
     */
    public static void debug(Object sender, String text) {
        if (Robot.verbosity != VerbosityLevel.DEBUG)
            return;
        System.out.println(getLogHeader(sender, "DEBUG") + text);
    }

    /**
     * Broadcasts a WARN message to the RIOLOG
     * @param sender The originating {@link Object}
     * @param text The {@link String} to broadcast.
     */
    public static void warn(Object sender, String text) {
        DriverStation.reportWarning(getLogHeader(sender, "WARN") + text, false);
    }

    /**
     * Broadcasts an ERROR message to the RIOLOG
     *
     * @param sender The originating {@link Object}
     * @param text The {@link String} to broadcast.
     */
    public static void error(Object sender, String text) {
        DriverStation.reportError(getLogHeader(sender, "ERROR") + text, false);
    }

    /**
     * Broadcasts an INFO message to the RIOLOG.
     *
     * @param sender The originating {@link Object}.
     * @param text   The {@link String} to broadcast.
     */
    public static void info(Object sender, String text) {
        System.out.println(getLogHeader(sender, "INFO") + text);
    }

    /**
     * Broadcasts a DEBUG message to the RIOLOG with a null sender.
     * @param text The {@link String} to broadcast.
     */
    @Deprecated(forRemoval = true)
    public static void debug(String text) { debug(null, text); }

    /**
     * Broadcasts an INFO message to the RIOLOG with a null sender.
     * @param text The {@link String} to broadcast.
     */
    @Deprecated(forRemoval = true)
    public static void info(String text) { info(null, text); }

    /**
     * Formats the uptime as a string.
     *
     * @param uptime The uptime in milliseconds.
     * @return The formatted uptime string.
     */
    private static String formatTime(long uptime) {
        long totalSeconds = uptime / 1000;
        long hours = totalSeconds / 3600;
        long minutes = (totalSeconds % 3600) / 60;
        long seconds = totalSeconds % 60;
        long millis = uptime % 1000;

        return String.format("%02d:%02d:%02d.%03d", hours, minutes, seconds, millis);
    }

    /**
     * Broadcasts an ERROR message to the RIOLOG if {@link REVLibError} is not OK.
     * @param sender The originating {@link Object}
     */
    public static void errorOnFail(Object sender, REVLibError error, String message) {
        if (error != REVLibError.kOk)
            if (message != null && !message.isEmpty())
                error(sender, message + " | " + error.name());
            else
                error(sender, error.name());
    }

    /**
     * Broadcasts an WARN message to the RIOLOG if {@link REVLibError} is not OK.
     * @param sender The originating {@link Object}
     */
    public static void warnOnFail(Object sender, REVLibError error, String message) {
        if (error != REVLibError.kOk)
            if (message != null && !message.isEmpty())
                warn(sender, message + " | " + error.name());
            else
                warn(sender, error.name());
    }

    public static void errorOnFail(REVLibError error, String message) { errorOnFail(null, error, message); }
    public static void warnOnFail(REVLibError error, String message) { warnOnFail(null, error, message); }
    public static void errorOnFail(REVLibError error) { errorOnFail(null, error, ""); }
    public static void warnOnFail(REVLibError error) { warnOnFail(null, error, ""); }

    public static void warn(String text) { warn(null, text); }
    public static void error(String text) { error(null, text); }

    private static final Function<AlertType, String[]> getEnabledAlerts = type -> {
        ArrayList<String> output = new ArrayList<>();

        // First, check for any conditional alerts which may be thrown.
        if (!ALERTS.isEmpty()) {
            Iterator<Alert> it = ALERTS.iterator();
            while (it.hasNext()) {
                Alert a = it.next();
                if (a.shouldRemove()) {
                    it.remove();
                    continue;
                }
                if ((!a.isEnabled() && System.currentTimeMillis() > testModeDisable) || a.getType() != type)
                    continue;
                output.add(a.getName());
            }
        }

        if (type == AlertType.ERROR && System.currentTimeMillis() <= testModeDisable) {
            output.add("----- DASHBOARD TEST: AUTO CLEAR -----");
        }

        return output.toArray(String[]::new);
    };

    public static synchronized void run() {
        if (System.currentTimeMillis() - lastLoopUpdate < 20)
            return;

        Iterator<Looper> it = LOOPS.iterator();
        while (it.hasNext()) {
            Looper looper = it.next();
            if (!looper.isFinished() && !looper.isRunning()) {
                looper.start();
                debug(IOManager.class, "Executing loop (\"" + looper.getName() + "\")");
                continue;
            }
            if (looper.isFinished()) {
                debug(IOManager.class, "Terminating loop (\"" + looper.getName() + "\")");
                it.remove();
                continue;
            }
            looper.run();
        }

        lastLoopUpdate = System.currentTimeMillis();

        //////////////////////////////////////////////////////////////////////////////////////////////////////////////
        if (System.currentTimeMillis() - lastAlertUpdate <= ALERT_PERIODIC_MS.get())
            return;

        Sendable alertData = builder -> {
            builder.setSmartDashboardType("Alerts");
            for (AlertType type : AlertType.DASHBOARD_ORDER) {
                builder.addStringArrayProperty(type.getPrefix(), () -> getEnabledAlerts.apply(type), null);
            }
        };

        SmartDashboard.putData("Alerts", alertData);
        lastAlertUpdate = System.currentTimeMillis();
    }
}
