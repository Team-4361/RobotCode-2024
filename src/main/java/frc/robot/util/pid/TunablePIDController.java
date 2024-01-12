package frc.robot.util.pid;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TunablePIDController extends PIDController {

    private String dashName;

    /**
     * Returns the next output of the PID controller.
     *
     * @param measurement The current measurement of the process variable.
     * @param setpoint    The new setpoint of the controller.
     * @return The next controller output.
     */
    @Override
    public double calculate(double measurement, double setpoint) {
        double output = super.calculate(measurement, setpoint);

        // Update the PID with the Dashboard.
        setPID(
                SmartDashboard.getNumber(dashName + ": P", getP()),
                SmartDashboard.getNumber(dashName + ": I", getI()),
                SmartDashboard.getNumber(dashName + ": D", getD())
        );

        return output;
    }

    public void resetDashboard() {
        SmartDashboard.putNumber(dashName + ": P", getP());
        SmartDashboard.putNumber(dashName + ": I", getI());
        SmartDashboard.putNumber(dashName + ": D", getD());
    }

    public TunablePIDController setDashboardName(String dashName) {
        this.dashName = dashName;
        resetDashboard();
        return this;
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd and a default period of
     * 0.02 seconds.
     *
     * @param kp The proportional coefficient.
     * @param ki The integral coefficient.
     * @param kd The derivative coefficient.
     */
    public TunablePIDController(String dashName, double kp, double ki, double kd) {
        super(kp, ki, kd);
        this.dashName = dashName;
        resetDashboard();
    }

    /**
     * Allocates a PIDController with the given constants for kp, ki, and kd.
     *
     * @param kp     The proportional coefficient.
     * @param ki     The integral coefficient.
     * @param kd     The derivative coefficient.
     * @param period The period between controller updates in seconds. Must be non-zero and positive.
     */
    public TunablePIDController(String dashName, double kp, double ki, double kd, double period) {
        super(kp, ki, kd, period);
        this.dashName = dashName;
        resetDashboard();
    }
}
