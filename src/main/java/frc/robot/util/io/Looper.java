package frc.robot.util.io;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import org.littletonrobotics.junction.Logger;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This {@link Looper} class is designed to be the bare-minimum for <code>periodic</code>
 * and <code>simulationPeriodic</code> methods. Unlike a {@link Subsystem},
 * this supports conditions such as maximum execution cycles, or
 * delay in-between calls — preventing a {@link Thread#sleep(long)} call.
 * <p></p>
 * <b>NOTICES:</b>
 * <ol>
 *     <li>
 *         You need to register <b>each</b> {@link Looper} in the {@link IOManager}
 *         class; this allows the system to run using the {@link IOManager#run()} method. If
 *         the <code>addToManager</code> option is called in the Constructor, this is done <b>automatically.</b>
 *     </li>
 *     <p></p>
 *     <li>
 *         All <code>periodic</code> and <code>simulationPeriodic</code> calls are <b>NOT</b>
 *         multi-threaded! It is <b>NOT</b> recommended to put any {@link Thread#sleep(long)}
 *         calls or equivalent, as the next {@link Looper} will be delayed. As an alternative,
 *         use the "interval" feature in this class.
 *     </li>
 * </ol>
 *
 * @author Eric Gold
 * @since 0.0.0
 * @version 0.0.2
 */
@SuppressWarnings("UnusedReturnValue")
public class Looper {
    private final ArrayList<Runnable> intervalCalls;
    private final ArrayList<Runnable> endCalls;
    private final ArrayList<Runnable> initCalls;
    private final String name;
    private final Notifier notifier;

    private long intervalMs;

    private boolean manFinished;
    private boolean running;

    /**
     * Starts the {@link Looper} instance <b>when managed by the {@link IOManager}
     * or equivalent.</b>
     *
     * @see IOManager#run()
     */
    public void start() {
        if (running)
            return;

        running = true;
        initCalls.forEach(Runnable::run);
        notifier.startPeriodic(intervalMs/1000f);
    }
    /**
     * Stops the {@link Looper} instance <b>when managed by the {@link IOManager}
     * or equivalent.</b> This method also resets the <code>nextMillis</code> to
     * account for a potential end delay.
     */
    public void stop() { manFinished = true; }

    /** @return If the current {@link Looper} instance is <b>running.</b> */
    public boolean isRunning() { return this.running; }

    /**
     * @return If the current {@link Looper} instance is <b>finished</b> and NOT running;
     * (ex. maximum execution cycles reached, or {@link #stop()} method is called.)
     */
    public boolean isFinished() {
        return !isRunning() && isRawFinished();
    }

    /**
     * @return If the {@link Looper} is <b>actually finished.</b> Unlike the {@link #isFinished()},
     * method, this does not care about the running status.
     */
    private boolean isRawFinished() { return manFinished; }

    /**
     * Adds a new Periodic {@link Runnable} to the {@link Looper} instance.
     * @param runnable The {@link Runnable} to add.
     * @return The current {@link Looper} instance with the updated changes.
     */
    public Looper addPeriodic(Runnable runnable) {
        intervalCalls.add(runnable);
        return this;
    }

    /**
     * Adds a new Initialize {@link Runnable} to the {@link Looper} instance.
     * @param runnable The {@link Runnable} to add.
     * @return The current {@link Looper} instance with the updated changes.
     */
    public Looper addInit(Runnable runnable) {
        initCalls.add(runnable);
        return this;
    }

    /**
     * Adds a new {@link Runnable} to the {@link Looper} instance which will be called after execution.
     * @param runnable The {@link Runnable} to add.
     * @return The current {@link Looper} instance with the updated changes.
     */
    public Looper addOnFinished(Runnable runnable) {
        endCalls.add(runnable);
        return this;
    }

    /**
     * Sets the {@link Duration} interval between <code>periodic</code>/<code>simulationPeriodic</code>
     * {@link Runnable} calls.
     *
     * @param intervalMs The {@link Duration} interval to use; <b>minimum of 20 milliseconds.</b>
     * @return The modified {@link Looper} instance.
     */
    public Looper setIntervalMs(long intervalMs) {
        this.intervalMs = intervalMs;
        return this;
    }

    /**
     * @return The {@link Duration} interval between
     * <code>periodic</code>/<code>simulationPeriodic</code>{@link Runnable} calls
     */
    public long getIntervalMs() { return this.intervalMs; }

    /** @return The current periodic {@link Runnable}s. */
    public ArrayList<Runnable> getPeriodicCalls() { return this.intervalCalls; }

    /** @return The current finished {@link Runnable}s. */
    public ArrayList<Runnable> getFinishedCalls() { return this.endCalls; }

    /** @return The current initialize {@link Runnable}s. */
    public ArrayList<Runnable> getInitCalls() { return this.initCalls; }

    /**
     * Constructs a new {@link Looper} with the input parameters, and is <b>automatically</b>
     * added to the {@link IOManager}.
     *
     * @param intervalMs    The duration between <code>periodic</code>/<code>simulationPeriodic</code>
     *                      {@link Runnable} calls.
     */
    Looper(String name, long intervalMs) {
        this.name = name;
        this.intervalMs = intervalMs;
        this.running = false;
        this.manFinished = false;

        this.intervalCalls = new ArrayList<>();
        this.endCalls = new ArrayList<>();
        this.initCalls = new ArrayList<>();
        this.notifier = new Notifier(this::run);
    }

    private void run() {
        if (isRawFinished()) {
            endCalls.forEach(Runnable::run);
            running = false;
            notifier.stop();
            return;
        }

        intervalCalls.forEach(Runnable::run);
    }

    /** @return The name of the {@link Looper}. */
    public String getName() { return this.name; }
}
