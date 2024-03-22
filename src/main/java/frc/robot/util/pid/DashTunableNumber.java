package frc.robot.util.pid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Consumer;

/**
 * This {@link DashTunableNumber} class provides the Driver/Operator with an easy-to-use way to
 * tune values <b>without re-deployment</b>. This allows for a significant speed boost in tuning!
 */
public class DashTunableNumber implements IUpdatable {
    private final String name;
    private final String dashString;
    private final ArrayList<Consumer<Double>> consumers;
    private double value;
    private long nextMillis;

    /**
     * Constructs a {@link DashTunableNumber} instance with the specified parameters.
     * @param name         The {@link String} name of the {@link DashTunableNumber}.
     * @param initialValue The initial {@link Double} of the {@link DashTunableNumber}
     * @param usePrefix    If the {@link SmartDashboard} entry should contain a prefix.
     */
    public DashTunableNumber(String name, double initialValue, boolean usePrefix) {
        this.name = name;
        this.value = initialValue;
        this.dashString = usePrefix ? name + ": Number" : name;
        this.consumers = new ArrayList<>();
        this.nextMillis = System.currentTimeMillis();

        SmartDashboard.putNumber(dashString, value);
    }

    /**
     * Constructs a {@link DashTunableNumber} instance with the specified parameters; <b>prefix is enabled.</b>
     * @param name         The {@link String} name of the {@link DashTunableNumber}.
     * @param initialValue The initial {@link Double} of the {@link DashTunableNumber}.
     */
    public DashTunableNumber(String name, double initialValue){
        this(name, initialValue, true);
    }

    /**
     * Adds a {@link Consumer} which allows the input of the saved {@link Double} value.
     * @param consumer The {@link Consumer} to add.
     */
    public void addConsumer(Consumer<Double> consumer) { consumers.add(consumer); }

    /** @return The current {@link String} name of the {@link DashTunableNumber}. */
    public String getName() { return this.name; }

    /** @return All registered {@link Consumer} instances in the {@link DashTunableNumber} */
    public ArrayList<Consumer<Double>> getConsumers() { return this.consumers; }

    /** @return The currently selected value. */
    public double getValue() { return this.value; }

    /** Updates the registered value with the {@link SmartDashboard} entry. <b>This method call is required!</b> */
    @Override
    public void update() {
        if (System.currentTimeMillis() < nextMillis)
            return;

        double temp = SmartDashboard.getNumber(dashString, value);
        if (temp != value)
            value = temp; // prevent constant re-assignment when not required.
        consumers.forEach(o -> o.accept(value));
        nextMillis = System.currentTimeMillis() + 1000;
    }
}