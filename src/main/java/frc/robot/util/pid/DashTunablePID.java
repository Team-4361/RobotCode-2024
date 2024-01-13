package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DashTunablePID {
    private final String name;
    private double kP, kI, kD;
    private boolean firstRun;
    private final String pStr, iStr, dStr;

    public String getName() { return this.name; }

    public DashTunablePID(String name, PIDConstants constants) {
        this.name = name;
        this.kP = constants.kP;
        this.kI = constants.kI;
        this.kD = constants.kD;
        this.firstRun = true;

        this.pStr = name + ": P";
        this.iStr = name + ": I";
        this.dStr = name + ": D";
    }

    public double getP() { return this.kP; }
    public double getI() { return this.kI; }
    public double getD() { return this.kD; }

    public void update() {
        if (firstRun) {
            SmartDashboard.putNumber(pStr, kP);
            SmartDashboard.putNumber(iStr, kI);
            SmartDashboard.putNumber(dStr, kD);
            firstRun = false;
            return;
        }

        // Not the first run, GET the values from the Dashboard.
        kP = SmartDashboard.getNumber(pStr, kP);
        kI = SmartDashboard.getNumber(iStr, kI);
        kD = SmartDashboard.getNumber(dStr, kD);
    }
}
