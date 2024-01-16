package frc.robot.util.io;

import java.util.Optional;
import java.util.function.Supplier;

/**
 * This {@link Alert} class represents a notification which can be triggered based
 * on specific conditions. Since a large {@link Object} quantity has the potential to clog the JVM/GC, a
 * <code>Simple Alert</code> can be used instead.
 *
 * @author Eric Gold
 * @since 0.0.1
 * @version 0.0.2
 */
public class Alert {
    private final AlertType type;
    private final String desc;
    private Supplier<Boolean> condition;

    private boolean persistent;
    private boolean oneUse;
    private boolean thrownPrev;
    private boolean manEnabled;
    private long enableDelay = 0;
    private long disableDelay = 0;
    private long enabledTime = -1;
    private long disabledTime = -1;

    /**
     * Constructs an Alert with the specified desc and {@link AlertType}.
     *
     * @param desc The Description of the alert.
     * @param type The {@link AlertType} of the alert.
     */
    Alert(String desc, AlertType type) {
        this.desc = desc;
        this.type = type;
        this.thrownPrev = false;
        this.manEnabled = false;
    }

    /** The {@link AlertType} of the {@link Alert}. */
    public AlertType getType() { return this.type; }

    /** @return The Description of the {@link Alert}. */
    public String getName() { return this.desc; }

    /** @return the ability for the {@link Alert} to expire after throwing. */
    public boolean isPersistent() { return this.persistent; }

    /** @return The minimum time the {@link #condition} needs to be enabled for the {@link Alert} to throw. */
    public long getEnableDelay() { return this.enableDelay; }

    /** @return The minimum time the {@link #condition} needs to be disabled for the {@link Alert} to expire. */
    public long getDisableDelay() { return this.disableDelay; }

    public Optional<Supplier<Boolean>> getCondition() { return Optional.ofNullable(condition); }

    public Alert setCondition(Supplier<Boolean> condition) {
        this.condition = condition;
        return this;
    }

    /** @return The Enabled Status of the {@link Alert}. */
    public boolean isEnabled() {
        boolean output;
        if (manEnabled) {
            output = true;
        } else if (condition == null) {
            output = false;
        } else {
            output = condition.get();
            if (output) {
                if (enabledTime <= 0) {
                    enabledTime = System.currentTimeMillis() + enableDelay;
                    thrownPrev = true;
                }
                output = System.currentTimeMillis() < enabledTime;
            } else {
                enabledTime = -1;
            }

            if (!persistent && output) {
                return true;
            } else if (output) {
                disabledTime = -1;
                return true;
            } else {
                if (disabledTime <= 0) {
                    disabledTime = System.currentTimeMillis() + disableDelay;
                }
                output = System.currentTimeMillis() < disabledTime;
            }
        }
        return output;
    }

    /**
     * Sets the Enabled Status of the {@link Alert} <b>independent of any Conditions.</b>
     * @param enabled The Enabled Status of the {@link Alert}.
     */
    public void setEnabled(boolean enabled) {
        this.manEnabled = enabled;
        if (enabled) {
            this.thrownPrev = true;
            this.enabledTime = System.currentTimeMillis();
        } else {
            this.enabledTime = -1;
        }
    }

    /**
     * Sets the minimum time the {@link #condition} needs to be disabled for the {@link Alert} to expire.
     *
     * @param millis The disable delay in milliseconds.
     */
    public Alert setDisableDelay(long millis) {
        this.disableDelay = millis;
        return this;
    }

    /**
     * Sets the minimum time the {@link #condition} needs to be enabled for the {@link Alert} to throw.
     *
     * @param millis The enable delay in milliseconds.
     */
    public Alert setEnableDelay(long millis) {
        this.enableDelay = millis;
        return this;
    }

    /**
     * Enables/disables the ability for the {@link Alert} to expire after throwing.
     * @param persist The value to use.
     */
    public Alert setPersistence(boolean persist) {
        this.persistent = persist;
        return this;
    }

    /**
     * Indicates the {@link IOManager} if this {@link Alert} should be removed and destroyed upon
     * becoming disabled AFTER THROWING. Use this functionality if your code contains
     * a loop to add Alerts when necessary.
     *
     * @param oneUse The auto-remove status.
     * @return           The current {@link Alert} instance.
     */
    public Alert setOneUse(boolean oneUse) {
        this.oneUse = oneUse;
        return this;
    }

    /** @return If the {@link IOManager} should auto-remove this {@link Alert} after disabling. */
    public boolean isOneUse() { return this.oneUse; }

    /**
     * @return If the {@link Alert} should be removed from the {@link IOManager}. Based upon the
     * {@link #setOneUse(boolean)} method.
     */
    public boolean shouldRemove() {
        return !isEnabled() && thrownPrev && oneUse;
    }

    /** @return The {@link System#currentTimeMillis()} of the enable time. <b>-1 if disabled.</b>*/
    public long getEnableMillis() { return this.enabledTime; }

    /** @return The {@link System#currentTimeMillis()} of the <b>last</b> disable time. <b>-1 if enabled.</b>*/
    public long getLastDisableMillis() { return this.disabledTime; }
}