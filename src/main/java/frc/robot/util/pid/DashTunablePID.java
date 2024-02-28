package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
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
     * @param pC The {@link Consumer} to use for the Proportional variable.
     * @param iC The {@link Consumer} to use for the Integral variable.
     * @param dC The {@link Consumer} to use for the Derivative variable.
     * @see DashTunableNumber#addConsumer(Consumer)
     */
    public void addConsumer(Consumer<Double> pC, Consumer<Double> iC, Consumer<Double> dC) {
        tuneP.addConsumer(pC);
        tuneI.addConsumer(iC);
        tuneD.addConsumer(dC);
    }

    /** @return The current Proportional value. */
    public double getP() { return tuneP.getValue(); }

    /** @return The current Integral value. */
    public double getI() { return tuneI.getValue(); }

    /** @return The current Derivative value. */
    public double getD() { return tuneD.getValue(); }

    /** @return The current {@link String} name for the {@link DashTunablePID}. */
    public String getName() { return this.name; }

    /**
     * Updates the {@link DashTunablePID}. <b>This method is required to be called!</b>
     * @see DashTunableNumber#update()
     */
    public void update() {
        tuneP.update();
        tuneI.update();
        tuneD.update();
    }
}
