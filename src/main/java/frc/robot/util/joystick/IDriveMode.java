package frc.robot.util.joystick;

public interface IDriveMode {
    double getX(double x);
    double getY(double y);
    double getTwist(double twist);
    String getName();
}
