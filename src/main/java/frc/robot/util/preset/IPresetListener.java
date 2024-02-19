package frc.robot.util.preset;

/**
 * This {@link IPresetListener} is designed to alert Mechanisms when their target setpoint value has been changed.
 *
 * @author Eric Gold
 * @since 0.0.1
 * @version 0.0.1
 * @param <T> The type parameter to use for the value.
 */
public interface IPresetListener<T> {
    /**
     * This method is called when a Preset has been changed.
     * @param mapName The Preset Name.
     * @param value The Preset Value.
     */
    void onPresetAdjust(String mapName, T value);
}
