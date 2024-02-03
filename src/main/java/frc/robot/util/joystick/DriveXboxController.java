package frc.robot.util.joystick;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

import static frc.robot.Constants.Control.DEAD_ZONE;

public class DriveXboxController extends DriveHIDBase {
    private final XboxController hid;

    /**
     * Constructs a {@link DriveXboxController}
     *
     * @param port          The USB port ID the HID is connected to.
     * @param xInverted     If the <b>Robot X-Axis</b> is inverted from: (- left, + right)
     * @param yInverted     If the <b>Robot Y-Axis</b> is inverted from: (- up, + down)
     * @param twistInverted If the <b>Robot Twist-Axis</b> is inverted from: (- left, + right)
     * @param deadband      The minimum value the HID will recognize.
     * @param primaryMode   The {@link IDriveMode}s to use by default.
     * @param extraModes    Optional additional {@link IDriveMode}s to recognize and switch to.
     */
    public DriveXboxController(int port,
                        boolean xInverted,
                        boolean yInverted,
                        boolean twistInverted,
                        double deadband,
                        IDriveMode primaryMode,
                        IDriveMode... extraModes) {
        super(port, xInverted, yInverted, twistInverted, deadband, primaryMode, extraModes);
        this.hid = new XboxController(port);
    }

    /**
     * Constructs a {@link DriveXboxController} with all Inversion set to <b>false</b> and deadband set to
     * {@link Constants.Control#DEAD_ZONE}.
     *
     * @param port          The USB port ID the HID is connected to.
     * @param primaryMode   The {@link IDriveMode}s to use by default.
     * @param extraModes    Optional additional {@link IDriveMode}s to recognize and switch to.
     */
    public DriveXboxController(int port, IDriveMode primaryMode, IDriveMode... extraModes) {
        this(port, false, false, false, DEAD_ZONE, primaryMode, extraModes);
    }

    @Override public XboxController getHID() { return hid; }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @return an event instance representing the left bumper's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftBumper(EventLoop)
     */
    public Trigger leftBumper() { return leftBumper(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the left bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right bumper's digital signal attached to the given loop.
     */
    public Trigger leftBumper(EventLoop loop) { return hid.leftBumper(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @return an event instance representing the right bumper's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightBumper(EventLoop)
     */
    public Trigger rightBumper() { return rightBumper(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the right bumper's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left bumper's digital signal attached to the given loop.
     */
    public Trigger rightBumper(EventLoop loop) { return hid.rightBumper(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @return an event instance representing the left stick button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #leftStick(EventLoop)
     */
    public Trigger leftStick() { return leftStick(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the left stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the left stick button's digital signal attached to the given loop.
     */
    public Trigger leftStick(EventLoop loop) { return hid.leftStick(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @return an event instance representing the right stick button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #rightStick(EventLoop)
     */
    public Trigger rightStick() { return rightStick(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the right stick button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the right stick button's digital signal attached to the given loop.
     */
    public Trigger rightStick(EventLoop loop) { return hid.rightStick(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @return an event instance representing the A button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #a(EventLoop)
     */
    public Trigger a() { return a(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the A button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the A button's digital signal attached to the given loop.
     */
    public Trigger a(EventLoop loop) { return hid.a(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @return an event instance representing the B button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #b(EventLoop)
     */
    public Trigger b() { return b(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the B button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the B button's digital signal attached to the given loop.
     */
    public Trigger b(EventLoop loop) { return hid.b(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @return an event instance representing the X button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #x(EventLoop)
     */
    public Trigger x() { return x(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the X button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the X button's digital signal attached to the given loop.
     */
    public Trigger x(EventLoop loop) { return hid.x(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @return an event instance representing the Y button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #y(EventLoop)
     */
    public Trigger y() { return y(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the Y button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the Y button's digital signal attached to the given loop.
     */
    public Trigger y(EventLoop loop) { return hid.y(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @return an event instance representing the start button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #start(EventLoop)
     */
    public Trigger start() { return start(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the start button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the start button's digital signal attached to the given loop.
     */
    public Trigger start(EventLoop loop) { return hid.start(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @return an event instance representing the back button's digital signal attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     * @see #back(EventLoop)
     */
    public Trigger back() { return back(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the back button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the back button's digital signal attached to the given loop.
     */
    public Trigger back(EventLoop loop) { return hid.back(loop).castTo(Trigger::new); }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param loop      the event loop instance to attach the Trigger to.
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided threshold, attached to
     * the given event loop
     */
    public Trigger leftTrigger(EventLoop loop, double threshold) {
        return hid.leftTrigger(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the left trigger's axis exceeds the provided threshold, attached to
     * the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger leftTrigger(double threshold) {
        return leftTrigger(CommandScheduler.getInstance().getDefaultButtonLoop(), threshold);
    }

    /**
     * Constructs a Trigger instance around the axis value of the left trigger. The returned trigger will be true when
     * the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the left trigger's axis exceeds 0.5, attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger leftTrigger() { return leftTrigger(0.5); }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @param loop      the event loop instance to attach the Trigger to.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided threshold, attached to
     * the given event loop
     */
    public Trigger rightTrigger(double threshold, EventLoop loop) {
        return hid.rightTrigger(threshold, loop).castTo(Trigger::new);
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than {@code threshold}.
     *
     * @param threshold the minimum axis value for the returned {@link Trigger} to be true. This value should be in the
     *                  range [0, 1] where 0 is the unpressed state of the axis.
     * @return a Trigger instance that is true when the right trigger's axis exceeds the provided threshold, attached to
     * the {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger rightTrigger(double threshold) {
        return rightTrigger(threshold, CommandScheduler.getInstance().getDefaultButtonLoop());
    }

    /**
     * Constructs a Trigger instance around the axis value of the right trigger. The returned trigger will be true when
     * the axis value is greater than 0.5.
     *
     * @return a Trigger instance that is true when the right trigger's axis exceeds 0.5, attached to the
     * {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger rightTrigger() { return rightTrigger(0.5); }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftX() { return getDriveMode().getY(hid.getLeftX()); }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightX() { return getDriveMode().getY(hid.getRightX()); }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return The axis value.
     */
    public double getLeftY() { return getDriveMode().getX(hid.getLeftY()); }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return The axis value.
     */
    public double getRightY() { return getDriveMode().getX(hid.getRightY()); }

    /**
     * Get the left trigger (LT) axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getLeftTriggerAxis() { return hid.getLeftTriggerAxis(); }

    /**
     * Get the right trigger (RT) axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return The axis value.
     */
    public double getRightTriggerAxis() { return hid.getRightTriggerAxis(); }

    /** @return The <b>raw</b> Robot X value without inversion. */
    @Override protected double getRawRobotX() { return getLeftY(); }

    /** @return The <b>raw</b> Robot Y value without inversion. */
    @Override protected double getRawRobotY() { return getLeftX(); }

    /** @return The <b>raw</b> Robot Twist value without inversion. */
    @Override protected double getRawRobotTwist() { return getRightX(); }
}