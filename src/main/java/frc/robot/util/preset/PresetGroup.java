package frc.robot.util.preset;

import java.util.ArrayList;
import java.util.List;


/**
 * This {@link PresetGroup} is designed to handle multiple {@link IPresetContainer} interfaces.
 *
 * @author Eric Gold
 * @since 0.0.1
 * @version 0.0.1
 */
public class PresetGroup extends ArrayList<IPresetContainer> implements IPresetContainer {
    private final String name;
    private int index;

    /**
     * Constructs a new {@link PresetGroup} with the specified Type and Name.
     *
     * @param name The name of the {@link PresetGroup}.
     */
    public PresetGroup(String name) {
        this.name = name;
        this.index = 0;
    }

    /**
     * Constructs a new {@link PresetGroup} with the specified Type and Name.
     *
     * @param name     The name of the {@link PresetGroup}.
     * @param elements The elements to add.
     */
    public PresetGroup(String name, IPresetContainer... elements) {
        this(name);
        addAll(List.of(elements));
    }

    /** @return The name of the {@link PresetGroup} */
    @Override public String getName() { return name; }

    /** The currently selected Preset Index. */
    @Override public int getSelectedIndex() { return index; }

    /** The maximum Preset Index which can be chosen. */
    @Override public int getMaxIndex() {
        return stream()
                .mapToInt(IPresetContainer::getMaxIndex)
                .max()
                .orElse(0);
    }

    /**
     * Attempts to set the Preset to the specific Index.
     *
     * @param idx The Index to change the Preset to.
     * @return True if the operation was successful; false otherwise.
     */
    @Override
    public boolean setPreset(int idx) {
        if (idx < 0 || idx > getMaxIndex())
            return false;

        this.index = idx;
        for (IPresetContainer c : this) {
            if (!c.setPreset(idx))
                return false;
        }
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
        for (IPresetContainer preset : this) {
            if (preset.getName().equalsIgnoreCase(name)) {
                return setPreset(idx);
            }
            idx++;
        }
        return false;
    }

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

    /** @return If the {@link IPresetContainer} has finished moving position. */
    @Override public boolean isFinished() { return true; }
}
