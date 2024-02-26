package frc.robot.util.joystick;

public class DriveMode {
    public static final IDriveMode LINEAR_MAP = new IDriveMode() {
        @Override public double getX(double x) { return x; }
        @Override public double getY(double y) { return y; }
        @Override public double getTwist(double twist) { return twist; }
        @Override public String getName() { return "Linear"; }
    };

    public static final IDriveMode SMOOTH_MAP = new IDriveMode() {
        @Override public double getX(double x) { return Math.pow(x, 3); }
        @Override public double getY(double y) { return Math.pow(y, 3); }
        @Override public double getTwist(double twist) { return Math.pow(twist, 3); }
        @Override public String getName() { return "Smooth"; }
    };

    public static final IDriveMode SLOW_MODE = new IDriveMode() {
        @Override public double getX(double x) { return SMOOTH_MAP.getX(x)/2; }
        @Override public double getY(double y) { return SMOOTH_MAP.getY(y)/2; }
        @Override public double getTwist(double twist) { return SMOOTH_MAP.getTwist(twist)/4;}
        @Override public String getName() { return "Slow Mode"; }
    };
}
