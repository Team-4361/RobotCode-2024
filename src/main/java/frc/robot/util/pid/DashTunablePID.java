package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Consumer;

public class DashTunablePID {
    private final String name;
    private final DashTunableNumber tuneP, tuneI, tuneD;

    /**
     * Constructs a new {@link DashTunablePID} with the specified parameters.
     * @param name      The name of the {@link DashTunablePID}.
     * @param constants The {@link PIDConstants} to initialize with.
     */
    public DashTunablePID(String name, PIDConstants constants) {
        this.name = name;
        tuneP = new DashTunableNumber(name + ": P", constants.kP, false);
        tuneI = new DashTunableNumber(name + ": I", constants.kI, false);
        tuneD = new DashTunableNumber(name + ": D", constants.kD, false);
    }

    /**
     * Adds a new {@link Consumer} set to the {@link DashTunablePID}.
     *
     * @param pC The {@link Consumer} to use for the P variable.
     * @param iC The {@link Consumer} to use for the I variable.
     * @param dC The {@link Consumer} to use for the D variable.
     */
    public void addConsumer(Consumer<Double> pC, Consumer<Double> iC, Consumer<Double> dC) {
        allPCs.add(pC);
        allDCs.add(dC);
        allICs.add(iC);
    }

    public double getP() { return this.kP; }
    public double getI() { return this.kI; }
    public double getD() { return this.kD; }

    public void update() {
        // Not the first run, GET the values from the Dashboard.
        kP = SmartDashboard.getNumber(pStr, kP);
        kI = SmartDashboard.getNumber(iStr, kI);
        kD = SmartDashboard.getNumber(dStr, kD);

        allPCs.forEach(o -> o.accept(kP));
        allICs.forEach(o -> o.accept(kI));
        allDCs.forEach(o -> o.accept(kD));
    }
}
