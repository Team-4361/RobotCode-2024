package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import java.util.function.Consumer;

public class TunablePID implements IUpdatable {
    private final String name;
    private final TunableNumber tuneP, tuneI, tuneD;

    /**
     * Constructs a new {@link TunablePID} with the specified parameters.
     * @param name      The name of the {@link TunablePID}.
     * @param constants The {@link PIDConstants} to initialize with.
     */
    public TunablePID(String name, PIDConstants constants, boolean enabled) {
        this.name = name;
        tuneP = new TunableNumber(name + ": P", constants.kP, enabled);
        tuneI = new TunableNumber(name + ": I", constants.kI, enabled);
        tuneD = new TunableNumber(name + ": D", constants.kD, enabled);
    }

    public void setEnabled(boolean enabled) {
        tuneP.setEnabled(enabled);
        tuneI.setEnabled(enabled);
        tuneD.setEnabled(enabled);
    }

    /**
     * Adds a new {@link Consumer} set to the {@link TunablePID}.
     *
     * @param pC The {@link Consumer} to use for the Proportional variable.
     * @param iC The {@link Consumer} to use for the Integral variable.
     * @param dC The {@link Consumer} to use for the Derivative variable.
     * @see TunableNumber#addConsumer(Consumer)
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

    /** @return The current {@link String} name for the {@link TunablePID}. */
    public String getName() { return this.name; }

    /**
     * Updates the {@link TunablePID}. <b>This method is required to be called!</b>
     * @see TunableNumber#update()
     */
    @Override
    public void update() {
        tuneP.update();
        tuneI.update();
        tuneD.update();
    }
}
