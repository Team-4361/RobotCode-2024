package frc.robot.util.preset;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import java.util.concurrent.atomic.AtomicReference;

/**
 * This interface enables {@link PresetMap} consistency and allows chaining of
 * multiple synchronized {@link PresetGroup}s -- similar logic to {@link SequentialCommandGroup} implementing
 * the {@link Command} interface.
 *
 * @author Eric Gold
 * @since 0.0.1
 */
public interface IPresetContainer {
    /** @return The name of the {@link IPresetContainer} */
    String getName();

    /** @return The currently selected Preset Index. */
    int getSelectedIndex();

    /** @return The maximum Preset Index which can be chosen. */
    int getMaxIndex();

    /**
     * Attempts to set the Preset to the specific Index.
     * @param idx The Index to change the Preset to.
     * @return True if the operation was successful; false otherwise.
     */
    boolean setPreset(int idx);

    /**
     * Advances the Preset Container to the next option.
     * @param loop If the Preset Container should loop to the first element if necessary.
     * @return True if the operation was successful; false otherwise.
     */
    boolean nextPreset(boolean loop);

    /**
     * Declines the Preset Container to the previous option.
     * @param loop If the Preset Container should loop to the last element if necessary.
     * @return True if the operation was successful; false otherwise.
     */
    boolean backPreset(boolean loop);

    /**
     * @return If the {@link IPresetContainer} has finished moving position; used for sequential effects.
     */
    boolean isFinished();
}
