package frc.robot.util.joystick;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.event.EventLoop;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;

public class DrivePS4Controller extends DriveHIDBase {
    private final PS4Controller hid;

    /**
     * Constructs a {@link DrivePS4Controller}
     *
     * @param port          The USB port ID the HID is connected to.
     * @param xInverted     If the <b>Robot X-Axis</b> is inverted from: (- left, + right)
     * @param yInverted     If the <b>Robot Y-Axis</b> is inverted from: (- up, + down)
     * @param twistInverted If the <b>Robot Twist-Axis</b> is inverted from: (- left, + right)
     * @param deadband      The minimum value the HID will recognize.
     * @param primaryMode   The {@link IDriveMode}s to use by default.
     * @param extraModes    Optional additional {@link IDriveMode}s to recognize and switch to.
     */
    public DrivePS4Controller(int port,
                              boolean xInverted,
                              boolean yInverted,
                              boolean twistInverted,
                              double deadband,
                              IDriveMode primaryMode,
                              IDriveMode... extraModes) {
        super(port, xInverted, yInverted, twistInverted, deadband, primaryMode, extraModes);
        this.hid = new PS4Controller(port);
    }

    /**
     * Constructs a {@link DrivePS4Controller} with all Inversion set to <b>false</b> and deadband set to
     * {@link Constants.Control#DEAD_ZONE}.
     *
     * @param port The USB port ID the HID is connected to.
     * @param primaryMode   The {@link IDriveMode}s to use by default.
     * @param extraModes    Optional additional {@link IDriveMode}s to recognize and switch to.
     */
    public DrivePS4Controller(int port, IDriveMode primaryMode, IDriveMode... extraModes) {
        super(port, primaryMode, extraModes);
        this.hid = new PS4Controller(port);
    }


    @Override public PS4Controller getHID() { return hid; }

    /**
     * Constructs an event instance around the L2 button's digital signal.
     *
     * @return an event instance representing the L2 button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger L2() { return L2(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the L2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L2 button's digital signal attached to the given
     *     loop.
     */
    public Trigger L2(EventLoop loop) { return hid.L2(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the R2 button's digital signal.
     *
     * @return an event instance representing the R2 button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger R2() { return R2(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the R2 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R2 button's digital signal attached to the given
     *     loop.
     */
    public Trigger R2(EventLoop loop) { return hid.R2(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the L1 button's digital signal.
     *
     * @return an event instance representing the L1 button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger L1() { return L1(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the L1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L1 button's digital signal attached to the given
     *     loop.
     */
    public Trigger L1(EventLoop loop) { return hid.L1(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the R1 button's digital signal.
     *
     * @return an event instance representing the R1 button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger R1() { return R1(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the R1 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R1 button's digital signal attached to the given
     *     loop.
     */
    public Trigger R1(EventLoop loop) { return hid.R1(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the L3 button's digital signal.
     *
     * @return an event instance representing the L3 button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger L3() { return L3(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the L3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the L3 button's digital signal attached to the given
     *     loop.
     */
    public Trigger L3(EventLoop loop) { return hid.L3(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the R3 button's digital signal.
     *
     * @return an event instance representing the R3 button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger R3() { return R3(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the R3 button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the R3 button's digital signal attached to the given
     *     loop.
     */
    public Trigger R3(EventLoop loop) { return hid.R3(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the square button's digital signal.
     *
     * @return an event instance representing the square button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger square() { return square(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the square button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the square button's digital signal attached to the given
     *     loop.
     */
    public Trigger square(EventLoop loop) { return hid.square(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the cross button's digital signal.
     *
     * @return an event instance representing the cross button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger cross() { return cross(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the cross button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the cross button's digital signal attached to the given
     *     loop.
     */
    public Trigger cross(EventLoop loop) { return hid.cross(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the triangle button's digital signal.
     *
     * @return an event instance representing the triangle button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger triangle() { return triangle(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the triangle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the triangle button's digital signal attached to the
     *     given loop.
     */
    public Trigger triangle(EventLoop loop) { return hid.triangle(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the circle button's digital signal.
     *
     * @return an event instance representing the circle button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger circle() { return circle(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the circle button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the circle button's digital signal attached to the given
     *     loop.
     */
    public Trigger circle(EventLoop loop) { return hid.circle(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the share button's digital signal.
     *
     * @return an event instance representing the share button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger share() { return share(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the share button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the share button's digital signal attached to the given
     *     loop.
     */
    public Trigger share(EventLoop loop) { return hid.share(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the PS button's digital signal.
     *
     * @return an event instance representing the PS button's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger PS() { return PS(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the PS button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the PS button's digital signal attached to the given
     *     loop.
     */
    public Trigger PS(EventLoop loop) { return hid.PS(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the options button's digital signal.
     *
     * @return an event instance representing the options button's digital signal attached to the
     *     {@link CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger options() { return options(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the options button's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the options button's digital signal attached to the
     *     given loop.
     */
    public Trigger options(EventLoop loop) { return hid.options(loop).castTo(Trigger::new); }

    /**
     * Constructs an event instance around the touchpad's digital signal.
     *
     * @return an event instance representing the touchpad's digital signal attached to the {@link
     *     CommandScheduler#getDefaultButtonLoop() default scheduler button loop}.
     */
    public Trigger touchpad() { return touchpad(CommandScheduler.getInstance().getDefaultButtonLoop()); }

    /**
     * Constructs an event instance around the touchpad's digital signal.
     *
     * @param loop the event loop instance to attach the event to.
     * @return an event instance representing the touchpad's digital signal attached to the given
     *     loop.
     */
    public Trigger touchpad(EventLoop loop) { return hid.touchpad(loop).castTo(Trigger::new); }

    /**
     * Get the X axis value of left side of the controller.
     *
     * @return the axis value.
     */
    public double getLeftX() { return hid.getLeftX(); }

    /**
     * Get the X axis value of right side of the controller.
     *
     * @return the axis value.
     */
    public double getRightX() { return hid.getRightX(); }

    /**
     * Get the Y axis value of left side of the controller.
     *
     * @return the axis value.
     */
    public double getLeftY() { return hid.getLeftY(); }

    /**
     * Get the Y axis value of right side of the controller.
     *
     * @return the axis value.
     */
    public double getRightY() { return hid.getRightY(); }

    /**
     * Get the L2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return the axis value.
     */
    public double getL2Axis() { return hid.getL2Axis(); }

    /**
     * Get the R2 axis value of the controller. Note that this axis is bound to the range of [0, 1] as
     * opposed to the usual [-1, 1].
     *
     * @return the axis value.
     */
    public double getR2Axis() { return hid.getR2Axis(); }

    /** @return The <b>raw</b> Robot X value without inversion. */
    @Override protected double getRawRobotX() { return getLeftX(); }

    /** @return The <b>raw</b> Robot Y value without inversion. */
    @Override protected double getRawRobotY() { return getLeftY(); }

    /** @return The <b>raw</b> Robot Twist value without inversion. */
    @Override protected double getRawRobotTwist() { return getRightX(); }
}
