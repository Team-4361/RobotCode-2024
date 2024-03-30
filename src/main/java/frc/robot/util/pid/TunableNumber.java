package frc.robot.util.pid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * This {@link TunableNumber} class provides the Driver/Operator with an easy-to-use way to
 * tune values <b>without re-deployment</b>. This allows for a significant speed boost in tuning!
 */
public class TunableNumber implements IUpdatable {
    private final String name;
    private final String dashString;
    private final ArrayList<Consumer<Double>> consumers;
    private double value;
    private long nextMillis;
    private boolean enabled;

    /**
     * Constructs a {@link TunableNumber} instance with the specified parameters.
     * @param name         The {@link String} name of the {@link TunableNumber}.
     * @param initialValue The initial {@link Double} of the {@link TunableNumber}
     */
    public TunableNumber(String name, double initialValue, boolean enabled) {
        this.name = name;
        this.value = initialValue;
        this.dashString = name.contains(":") ? name : name + ": Value";
        this.consumers = new ArrayList<>();
        this.nextMillis = System.currentTimeMillis();
        this.enabled = enabled;

        if (enabled)
            SmartDashboard.putNumber(dashString, value);
    }

    public void setValue(double value) {
        this.value = value;
        if (enabled)
            SmartDashboard.putNumber(dashString, value);
        nextMillis = System.currentTimeMillis() + 1000;
    }

    public void setEnabled(boolean enabled) { this.enabled = enabled; }
    public boolean isEnabled() { return this.enabled; }

    /**
     * Constructs a {@link TunableNumber} instance with the specified parameters; <b>prefix is enabled.</b>
     * @param name         The {@link String} name of the {@link TunableNumber}.
     * @param initialValue The initial {@link Double} of the {@link TunableNumber}.
     */
    public TunableNumber(String name, double initialValue){
        this(name, initialValue, true);
    }

    /**
     * Adds a {@link Consumer} which allows the input of the saved {@link Double} value.
     * @param consumer The {@link Consumer} to add.
     */
    public void addConsumer(Consumer<Double> consumer) { consumers.add(consumer); }

    /** @return The current {@link String} name of the {@link TunableNumber}. */
    public String getName() { return this.name; }

    /** @return All registered {@link Consumer} instances in the {@link TunableNumber} */
    public ArrayList<Consumer<Double>> getConsumers() { return this.consumers; }

    /** @return The currently selected value. */
    public double getValue() { return this.value; }

    /** Updates the registered value with the {@link SmartDashboard} entry. <b>This method call is required!</b> */
    @Override
    public void update() {
        if (!enabled || System.currentTimeMillis() < nextMillis)
            return;

        double temp = SmartDashboard.getNumber(dashString, value);
        if (temp != value)
            value = temp; // prevent constant re-assignment when not required.
        consumers.forEach(o -> o.accept(value));
        nextMillis = System.currentTimeMillis() + 1000;
    }
}