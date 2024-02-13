package frc.robot.util.preset;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * This {@link PresetMap} is designed to hold various fixed positions of a mechanism. Functionality
 * is almost identical to a HashMap, except for (a) elements are stored in a consistent order -- and (b)
 * the index is saved and noted with various supporting methods.
 *
 * @param <T> The type of values to hold.
 * @since 0.0.1
 * @version 0.0.1
 * @author Eric Gold
 */
public class PresetMap<T> extends LinkedHashMap<String, T> implements IPresetContainer {
    private final String name;
    private final ArrayList<IPresetListener<T>> listeners;
    private Supplier<Boolean> completeSupplier;
    private boolean dashboardEnabled;
    private long completeDelayMs;
    private long startMs = 0;
    private int index;

    public Supplier<T> getSupplier() { return this::getSelectedValue; }

    /**
     * Constructs a new {@link PresetMap} with the specified Type and Name.
     * @param name             The name of the {@link PresetMap}.
     * @param dashboardEnabled If the {@link PresetMap} should be logged to {@link SmartDashboard}.
     */
    public PresetMap(String name, boolean dashboardEnabled) {
        this.name = name;
        this.index = 0;
        this.listeners = new ArrayList<>();
        this.completeSupplier = null;
        this.completeDelayMs = 0;
        setDashboardEnabled(dashboardEnabled);
    }

    /**
     * Constructs a new {@link PresetMap} with the specified Type and Name.
     * <b>It will be logged to {@link SmartDashboard}</b>
     *
     * @param name The name of the {@link PresetMap}.
     */
    public PresetMap(String name) { this(name, true); }

    /**
     * Constructs a new {@link PresetMap} with the specified Type and Name.
     * @param name             The name of the {@link PresetMap}.
     * @param dashboardEnabled If the {@link PresetMap} should be logged to {@link SmartDashboard}.
     * @param elements         The elements to add.
     */
    public PresetMap(String name, boolean dashboardEnabled, LinkedHashMap<String, T> elements) {
        this(name, dashboardEnabled);
        this.putAll(elements);
    }

    /**
     * Constructs a new {@link PresetMap} with the specified Type and Name.
     * <b>It will be logged to {@link SmartDashboard}</b>
     *
     * @param name     The name of the {@link PresetMap}.
     * @param elements The elements to add.
     */
    public PresetMap(String name, LinkedHashMap<String, T> elements) {
        this(name, true, elements);
    }

    /** @return If this {@link PresetMap} will be logged to {@link SmartDashboard}. */
    public boolean isDashboardEnabled() { return this.dashboardEnabled; }

    /**
     * Sets if the {@link PresetMap} should be logged to {@link SmartDashboard}.
     * @param value The {@link Boolean} value to use.
     */
    public void setDashboardEnabled(boolean value) {
        this.dashboardEnabled = value;
        if (value)
            SmartDashboard.putString(getName(), getSelectedName());
    }

    /**
     * Sets the {@link Supplier} used to determine if {@link #isFinished()} is true.
     * @param supplier The Boolean {@link Supplier} to use.
     * @return The current {@link PresetMap} with the modification.
     */
    public PresetMap<T> setFinishedSupplier(Supplier<Boolean> supplier) {
        this.completeSupplier = supplier;
        return this;
    }

    /**
     * Sets the Delay used to determine if {@link #isFinished()} is true;
     * @param millis The Delay in milliseconds.
     * @return The current {@link PresetMap} with the modification.
     */
    public PresetMap<T> setFinishedDelay(long millis) {
        this.completeDelayMs = millis;
        return this;
    }

    /** @return The <b>optional</b> {@link Supplier} used to determine if {@link #isFinished()} is true. */
    public Optional<Supplier<Boolean>> getFinishedSupplier() {
        return Optional.ofNullable(completeSupplier);
    }

    /**
     * @return The delay used to determine if {@link #isFinished()} is true;
     * may be combined with {@link #getFinishedSupplier()}.
     */
    public long getFinishedDelay() { return this.completeDelayMs; }

    /**
     * Adds a {@link IPresetListener} to the {@link PresetMap}, and calls the
     * {@link IPresetListener#onPresetAdjust(String, Object)} method.
     *
     * @param listener The {@link IPresetListener} to add.
     * @return The current {@link PresetMap} with the modification.
     */
    @SuppressWarnings("UnusedReturnValue")
    public PresetMap<T> addListener(IPresetListener<T> listener) {
        this.listeners.add(listener);
        listener.onPresetAdjust(getSelectedName(), getSelectedValue());
        return this;
    }

    /**
     * @param idx The index to check for.
     * @return The assigned {@link String} name for the Preset or empty {@link String} if invalid.
     */
    public String getSelectedName(int idx) {
        int i = 0;
        if (idx < size()) {
            for (String tName : keySet()) {
                if (i == idx) {
                    return tName;
                }
                i++;
            }
        }
        return "";
    }

    /** @return The name of the {@link IPresetContainer} */
    @Override public String getName() { return name; }

    /** @return The currently selected Preset Index. */
    @Override public int getSelectedIndex() { return index; }

    /** @return The currently selected value. */
    public T getSelectedValue() { return get(getSelectedName()); }

    /** The maximum Preset Index which can be chosen. */
    @Override public int getMaxIndex() { return size()-1; }

    /**
     * Attempts to set the Preset to the specific Index.
     *
     * @param idx The Index to change the Preset to.
     * @return True if the operation was successful; false otherwise.
     */
    @Override
    public boolean setPreset(int idx) {
        if (idx >= size())
            return false;
        this.index = idx;
        this.startMs = System.currentTimeMillis();
        listeners.forEach(o -> o.onPresetAdjust(getSelectedName(), get(getSelectedName())));
        if (dashboardEnabled)
            SmartDashboard.putString(getName(), getSelectedName() + " index is " + index);
        return true;
    }

    /**
     * Attempts to set the Preset to the specific Name.
     *
     * @param name The Name to change the Preset to.
     * @return True if the operation was successful; false otherwise.
     */
    public boolean setPreset(String name) {
        int idx = 0;
        for (String tName : keySet()) {
            if (name.equalsIgnoreCase(tName))
                return setPreset(idx);
            idx++;
        }
        return false;
    }

    public void setPresets(LinkedHashMap<String, T> map) {
        clear();
        putAll(map);
    }

    /** @return The name of the currently selected Preset */
    public String getSelectedName() { return getSelectedName(index); }

    /**
     * Advances the Preset Container to the next option.
     * @return True if the operation was successful; false otherwise.
     */
    @Override
    public boolean nextPreset(boolean loop) {
        if (index + 1 < size()) return setPreset(index+1);
        if (loop) return setPreset(0);
        return false;
    }

    /**
     * Declines the Preset Container to the previous option.
     * @return True if the operation was successful; false otherwise.
     */
    @Override
    public boolean backPreset(boolean loop) {
        if (index - 1 >= 0) return setPreset(index-1);
        if (loop) return setPreset(getMaxIndex());
        return false;
    }

    /**
     * This method calculates the result based on the Supplier and/or Delay. Here are a few examples:
     * <ul>
     *     <li>
     *         Supplier AND 5000ms --> Supplier MUST be TRUE for AT LEAST 5000ms.
     *     </li>
     *     <li>
     *         1000ms with NO Supplier --> Preset must be set for AT LEAST 1000ms.
     *     </li>
     *     <li>
     *         Supplier AND NO DELAY --> Supplier MUST be TRUE.
     *     </li>
     *     <li>
     *         No Supplier OR Delay --> Returns true if preset has been set
     *         with {@link #setPreset(int)} or similar; false otherwise.
     *     </li>
     * </ul>
     * @return If the {@link IPresetContainer} has finished moving position; used for sequential effects.
     */
    @Override
    public boolean isFinished() {
        if (startMs == 0)
            return false; // no preset has been set yet.
        long elapsedTime = System.currentTimeMillis() - startMs;

        if (completeDelayMs > 0 && completeSupplier != null) { // Both delay and supplier are applicable
            return elapsedTime >= completeDelayMs && completeSupplier.get(); // Require supplier to be enabled for delayMs
        } else if (completeDelayMs > 0) { // Only delay is applicable
            return elapsedTime >= completeDelayMs;
        } else if (completeSupplier != null) { // Only supplier is applicable
            return completeSupplier.get();
        } else {
            // Neither condition is applicable, consider it finished
            return true;
        }
    }
}
