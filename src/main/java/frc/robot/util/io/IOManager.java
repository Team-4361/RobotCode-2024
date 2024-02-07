package frc.robot.util.io;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.REVLibError;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.pid.DashTunablePID;

import java.lang.management.ManagementFactory;
import java.lang.management.RuntimeMXBean;
import java.util.*;
import java.util.function.Function;

import static frc.robot.Constants.AlertConfig.ALERT_PERIODIC_MS;
import static frc.robot.Constants.AlertConfig.STRING_HIGH_PERIODIC_MS;
import static frc.robot.Constants.Debug.DEBUG_LOGGING_ENABLED;
import static frc.robot.Constants.LooperConfig.*;

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
    private static final List<DashTunablePID> PID_TUNES = new ArrayList<>();
    private static final RuntimeMXBean MX_BEAN = ManagementFactory.getRuntimeMXBean();

    private static long lastLoopUpdate = 0;
    private static long lastAlertUpdate = 0;

    private static final long testModeDisable = System.currentTimeMillis() + 5000;

    /** @return A {@link List} of all registered {@link Looper} instances. */
    public static List<Looper> getLoops() { return LOOPS; }

    /** @return A {@link List} of all registered {@link Alert} instances. */
    public static List<Alert> getAlerts() { return ALERTS; }

    /** @return A {@link List} of all registered {@link DashTunablePID} instances. */
    public static List<DashTunablePID> getPIDTunes() { return PID_TUNES; }

    static {
        getAlert(STRING_HIGH_PERIODIC_MS, AlertType.WARNING)
                .setCondition(() -> System.currentTimeMillis() - lastLoopUpdate >= 25)
                .setEnableDelay(2000)
                .setDisableDelay(2000)
                .setPersistent(false)
                .setOneUse(false);
    }

    /**
     * Initializes a {@link Notifier} instance and automatically registers it.
     *
     * @param loopName      The {@link String} name of the {@link Looper} (case-insensitive)
     * @param interval      The interval of the {@link Looper} <code>periodic</code> calls.
     *
     * @return True if the {@link Looper} does <b>not</b> exist and successfully initialized; false otherwise.
     */
    @SuppressWarnings("UnusedReturnValue")
    public static boolean initLoop(String loopName, long interval) {
        if (loopExists(loopName))
            return false;
        Looper looper = new Looper(loopName, interval);
        return LOOPS.add(looper);
    }

    /** @return If the {@link Looper} exists. <b>Highly recommended to call this before {@link #getLoop(String)}</b> */
    public static boolean loopExists(String loopName) { return (getLoop(loopName).isPresent()); }

    /** @return If the {@link DashTunablePID} exists. <b>Highly recommended to call this before a method.</b> */
    public static boolean tuneExists(String loopName) { return (getDashTune(loopName).isPresent()); }

    /**
     * Automatically registers a {@link DashTunablePID} instance based on the {@link PIDController} supplied
     * values. This {@link IOManager} also takes care of updating the PID instance.
     *
     * @param name The {@link String} name of the {@link DashTunablePID}.
     * @param controller The {@link PIDController} to pull the values from.
     * @return True if the {@link DashTunablePID} did not exist; false otherwise.
     */
    @SuppressWarnings("UnusedReturnValue")
    public static boolean initPIDTune(String name, PIDController controller) {
        if (tuneExists(name))
            return false;
        DashTunablePID pid = new DashTunablePID(name, new PIDConstants(
                controller.getP(),
                controller.getI(),
                controller.getD()
        ));
        pid.addConsumer(controller::setP, controller::setI, controller::setD); // will auto-update the controller.
        PID_TUNES.add(pid);

        // Attempt to add the update method to the loop.
        if (!loopExists(STRING_DASHBOARD_NAME))
            initLoop(STRING_DASHBOARD_NAME, DASHBOARD_INTERVAL);

        return addPeriodicIfExists(STRING_ODOMETRY_NAME, pid::update);
    }

    /**
     * Automatically registers a {@link DashTunablePID} instance based on the {@link SparkPIDController} supplied
     * values. This {@link IOManager} also takes care of updating the PID instance.
     * <p></p>
     * <b>If the tune already exists, it will simply be appended!</b>
     *
     * @param name The {@link String} name of the {@link DashTunablePID}.
     * @param controller The {@link SparkPIDController} to pull the values from.
     */
    public static void initPIDTune(String name, SparkPIDController controller) {
        Optional<DashTunablePID> tune = getDashTune(name);

        if (tune.isPresent()) {
            // Simply add the consumer to the tune. It already exists.
            tune.get().addConsumer(controller::setP, controller::setI, controller::setD);
            return;
        }

        DashTunablePID pid = new DashTunablePID(name, new PIDConstants(
                controller.getP(),
                controller.getI(),
                controller.getD()
        ));
        pid.addConsumer(controller::setP, controller::setI, controller::setD); // will auto-update the controller.
        PID_TUNES.add(pid);

        // Attempt to add the update method to the loop.
        if (!loopExists(STRING_DASHBOARD_NAME))
            initLoop(STRING_DASHBOARD_NAME, DASHBOARD_INTERVAL);

        addPeriodicIfExists(STRING_ODOMETRY_NAME, pid::update);
    }

    /**
     * Adds a <code>periodic</code> {@link Runnable} to the {@link Looper} only if currently initialized.
     * @param loopName The {@link String} name of the {@link Looper} (case-insensitive)
     * @param periodic The <code>periodic</code> {@link Runnable} to add.
     * @return True if the {@link Looper} exists; false otherwise.
     */
    @SuppressWarnings("UnusedReturnValue")
    public static boolean addPeriodicIfExists(String loopName, Runnable periodic) {
        Optional<Looper> loop;
        if ((loop = getLoop(loopName)).isEmpty()) {
            return false;
        }
        loop.get().addPeriodic(periodic);
        return true;
    }

    /**
     * Adds an <code>init</code> {@link Runnable} to the {@link Looper} only if currently initialized.
     * @param loopName The {@link String} name of the {@link Looper} (case-insensitive)
     * @param init     The <code>init</code> {@link Runnable} to add.
     * @return True if the {@link Looper} exists; false otherwise.
     */
    public static boolean addInitIfExists(String loopName, Runnable init) {
        Optional<Looper> loop;
        if ((loop = getLoop(loopName)).isEmpty()) {
            return false;
        }
        loop.get().addInit(init);
        return true;
    }

    /**
     * Adds an <code>onFinished</code> {@link Runnable} to the {@link Looper} only if currently initialized.
     * @param loopName  The {@link String} name of the {@link Looper} (case-insensitive)
     * @param onFinished The <code>onFinished</code> {@link Runnable} to add.
     * @return True if the {@link Looper} exists; false otherwise.
     */
    public static boolean addOnFinishedIfExists(String loopName, Runnable onFinished) {
        Optional<Looper> loop;
        if ((loop = getLoop(loopName)).isEmpty()) {
            return false;
        }
        loop.get().addOnFinished(onFinished);
        return true;
    }

    /**
     * Attempts to find a {@link Looper} based on the {@link String} name.
     * @param loopName The {@link String} name of the {@link Looper} (case-insensitive)
     * @return An {@link Optional} containing the {@link Looper} or empty if non-existent.
     */
    public static Optional<Looper> getLoop(String loopName) {
        return LOOPS.stream()
                .filter(o -> o.getName().equalsIgnoreCase(loopName))
                .findFirst();
    }

    /**
     * Attempts to find a {@link DashTunablePID} based on the {@link String} name.
     * @param pidName The {@link String} name of the {@link DashTunablePID} (case-insensitive)
     * @return An {@link Optional} containing the {@link DashTunablePID} or empty if non-existent.
     */
    public static Optional<DashTunablePID> getDashTune(String pidName) {
        return PID_TUNES.stream()
                .filter(o -> o.getName().equalsIgnoreCase(pidName))
                .findFirst();
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

    @SuppressWarnings("UnusedReturnValue")
    public static boolean deleteAlert(String alertName, AlertType type) {
        Iterator<Alert> it = ALERTS.iterator();
        while (it.hasNext()) {
            Alert alert = it.next();
            if (alert.getName().equalsIgnoreCase(alertName) && alert.getType() == type) {
                it.remove();
                return true;
            }
        }
        return false;
    }

    @SuppressWarnings("UnusedReturnValue")
    public static boolean deleteLoop(String loopName) {
        Iterator<Looper> it = LOOPS.iterator();
        while (it.hasNext()) {
            Looper loop = it.next();
            if (loop.getName().equalsIgnoreCase(loopName)) {
                // Try to force the Looper to end itself and call the "onFinished" method.
                loop.stop();
                it.remove();
                return true;
            }
        }
        return false;
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
        if (DEBUG_LOGGING_ENABLED)
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

    /**
     * Acknowledges an {@link Alert} by <b>deleting</b> the most recent one displayed to
     * the {@link Shuffleboard}. This method can be mapped to any button on a {@link Joystick}.
     * <p></p>
     * NOTE: This does <b>NOT</b> clear {@link Alert}s marked as <u>persistent!</u>
     * @return True if the operation succeeded; false otherwise.
     */
    @SuppressWarnings("UnusedReturnValue")
    public static boolean acknowledgeAlert() {
        return ALERTS.stream()
                .filter(o -> !o.isPersistent() && o.isEnabled() && o.getEnableMillis() > 0)
                .max(Comparator.comparingLong(Alert::getEnableMillis))
                .map(ALERTS::remove)
                .orElse(false);
    }

    /** @return A {@link Command} used to acknowledge an {@link Alert}. */
    public static Command acknowledgeAlertCommand() { return Commands.runOnce(IOManager::acknowledgeAlert); }

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
                info(IOManager.class, "Executing loop (\"" + looper.getName() + "\")");
                continue;
            }
            if (looper.isFinished()) {
                info(IOManager.class, "Terminating loop (\"" + looper.getName() + "\")");
                it.remove();
                continue;
            }
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
