package frc.robot.util.joystick;

import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import frc.robot.Constants;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.preset.IPresetContainer;

import java.util.ArrayList;

import static frc.robot.Constants.Control.DEADBAND;

/**
 * This {@link DriveHIDBase} class is designed to be a foundation for Drive Controllers. Unlike
 * a regular Xbox/Joystick/PS4 controller, this supports automatic inversions, Robot-coordinate conversions,
 * and {@link DriveMode}s to reduce sensitivity and increase smoothness. This {@link DriveHIDBase} can
 * be used for <b>mechanisms or driving.</b>
 * <hr>
 * Finally, this {@link DriveHIDBase} has the bonus of being fully compatible with
 * the {@link IPresetContainer} (a.k.a. Presets) for it's {@link DriveMode}s. This allows
 * for fine-grained control without repeating code.
 *
 * @author Eric Gold
 * @since 0.0.1
 * @version 0.0.1
 */
public abstract class DriveHIDBase extends CommandGenericHID implements IPresetContainer {
    private final ArrayList<IDriveMode> modes;
    private final boolean xInverted;
    private final boolean yInverted;
    private final boolean twistInverted;
    private final int port;

    private int index;
    private double deadband;

    /**
     * Constructs a {@link DriveHIDBase}
     *
     * @param port          The USB port ID the HID is connected to.
     * @param xInverted     If the <b>Robot X-Axis</b> is inverted from: (- left, + right)
     * @param yInverted     If the <b>Robot Y-Axis</b> is inverted from: (- up, + down)
     * @param twistInverted If the <b>Robot Twist-Axis</b> is inverted from: (- left, + right)
     * @param deadband      The minimum value the HID will recognize.
     * @param primaryMode   The {@link IDriveMode}s to use by default.
     * @param extraModes    Optional additional {@link IDriveMode}s to recognize and switch to.
     */
    public DriveHIDBase(int port,
                        boolean xInverted,
                        boolean yInverted,
                        boolean twistInverted,
                        double deadband,
                        IDriveMode primaryMode,
                        IDriveMode... extraModes) {
        super(port);
        this.port = port;
        this.xInverted = xInverted;
        this.yInverted = yInverted;
        this.twistInverted = twistInverted;
        this.deadband = deadband;
        this.index = 0;
        this.modes = new ArrayList<>();
        modes.add(primaryMode);
        for (IDriveMode mode : extraModes) {
            if (!modes.contains(mode))
                modes.add(mode);
        }
    }

    /**
     * Constructs a {@link DriveHIDBase} with all Inversion set to <b>false</b> and deadband set to
     * {@link Constants.Control#DEADBAND}.
     *
     * @param port          The USB port ID the HID is connected to.
     * @param primaryMode   The {@link IDriveMode}s to use by default.
     * @param extraModes    Optional additional {@link IDriveMode}s to recognize and switch to.
     */
    public DriveHIDBase(int port, IDriveMode primaryMode, IDriveMode... extraModes) {
        this(port, false, false, false, DEADBAND, primaryMode, extraModes);
    }

    /**
     * Registers a {@link IDriveMode} to the {@link DriveHIDBase} instance. Can
     * be switched in a Preset-like fashion.
     *
     * @param mode The {@link IDriveMode}
     * @return The modified {@link DriveHIDBase} instance.
     * @see IPresetContainer
     */
    public DriveHIDBase addDriveMode(IDriveMode mode) {
        modes.add(mode);
        return this;
    }

    /** @return The currently selected {@link IDriveMode}. */
    public IDriveMode getDriveMode() { return modes.get(index); }

    /** @return The assigned port of the {@link DriveHIDBase} */
    public int getPort() { return this.port; }

    /** @return The name of the {@link IPresetContainer} */
    @Override public String getName() { return port + " | Drive Modes"; }

    /** @return The currently selected Preset Index. */
    @Override public int getSelectedIndex() { return index; }

    /** @return The maximum Preset Index which can be chosen. */
    @Override public int getMaxIndex() { return modes.size()-1; }

    /** @return The name of the currently selected Preset */
    public String getPresetName() { return modes.get(index).getName(); }

    /**
     * @param idx The index to check for.
     * @return The assigned {@link String} name for the Preset or empty {@link String} if invalid.
     */
    public String getPresetName(int idx) {
        if (idx < 0 || idx > getMaxIndex())
            return "";
        return modes.get(idx).getName();
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
        index = idx;
        return true;
    }

    /**
     * Advances the Preset Container to the next option.
     *
     * @return True if the operation was successful; false otherwise.
     */
    @Override
    public boolean nextPreset(boolean loop) {
        if (index + 1 <= modes.size() - 1) {
            return setPreset(index+1);
        }
        if (loop)
            return setPreset(0);
        return false;
    }

    /**
     * Declines the Preset Container to the previous option.
     *
     * @return True if the operation was successful; false otherwise.
     */
    @Override
    public boolean backPreset(boolean loop) {
        if (index - 1 >= 0) {
            return setPreset(index-1);
        }
        if (loop)
            return setPreset(getMaxIndex());
        return false;
    }

    /**
     * @return If the {@link IPresetContainer} has finished moving position; used for sequential effects.
     */
    @Override public boolean isFinished() { return true; }

    /** @return The <b>raw</b> Robot X value (+ forward, - back) without inversion. */
    protected abstract double getRawRobotX();

    /** @return The <b>raw</b> Robot Y value (+ left, - right) without inversion. */
    protected abstract double getRawRobotY();

    /** @return The <b>raw</b> Robot Twist value (+ left, - right) without inversion. */
    protected abstract double getRawRobotTwist();

    /**
     * Sets the minimum value the HID will recognize.
     *
     * @param value The {@link Double} value to use.
     * @return The modified {@link DriveHIDBase} instance.
     */
    public DriveHIDBase setDeadband(double value) {
        this.deadband = value;
        return this;
    }

    /** @return The minimum value the HID will recognize. */
    public double getDeadband() { return this.deadband; }

    /** @return If the HID X-Axis is inverted. */
    public boolean isXAxisInverted() { return this.xInverted; }

    /** @return If the HID Y-Axis is inverted. */
    public boolean isYAxisInverted() { return this.yInverted; }

    /** @return If the HID Twist-Axis is inverted. */
    public boolean isTwistAxisInverted() { return this.twistInverted; }

    /** @return The X-axis (-1.0 to +1.0) using the robot-coordinate system. (+X forward, +Y left) */
    public double getRobotX() {
        double val = xInverted ? -getRawRobotX() : getRawRobotX();
        return ExtendedMath.deadband(modes.get(index).getX(val), deadband);
    }

    /** @return The Y-axis (-1.0 to +1.0) using the robot-coordinate system. (+X forward, +Y left) */
    public double getRobotY() {
        double val = yInverted ? -getRawRobotY() : getRawRobotY();
        return ExtendedMath.deadband(modes.get(index).getY(val), deadband);
    }

    /** @return The Twist-axis (-1.0 to +1.0) using the robot-coordinate system. (+X forward, +Y left) */
    public double getRobotTwist() {
        double val = twistInverted ? -getRawRobotTwist() : getRawRobotTwist();
        return ExtendedMath.deadband(modes.get(index).getTwist(val), deadband);
    }
}