package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import frc.robot.Constants;
import frc.robot.Constants.OperationMode;
import frc.robot.Robot;

import static frc.robot.Constants.Control.OP_MODE;

/**
 * This {@link PIDConstantsAK} class is designed to easily switch PID values with various
 * {@link OperationMode}s of the {@link Robot}.
 */
public class PIDConstantsAK {

    public final double kP;
    public final double kI;
    public final double kD;

    /**
     * Constructs a new {@link PIDConstantsAK} with the following parameters.
     *
     * @param realKP The P value for <code>REAL</code> mode.
     * @param realKI The I value for <code>REAL</code> mode.
     * @param realKD The D value for <code>REAL</code> mode.
     *
     * @param simKP  The P value for <code>SIM</code> mode.
     * @param simKI  The I value for <code>SIM</code> mode.
     * @param simKD  The D value for <code>SIM</code> mode.
     *
     * @param replayKP The P value for <code>REPLAY</code> mode.
     * @param replayKI The I value for <code>REPLAY</code> mode.
     * @param replayKD The D value for <code>REPLAY</code> mode.
     */
    public PIDConstantsAK(
            double realKP,   double realKI,   double realKD,
            double simKP,    double simKI,    double simKD,
            double replayKP, double replayKI, double replayKD
    ) {
        switch (OP_MODE) {
            case REAL: {
                kP = realKP;
                kI = realKI;
                kD = realKD;
                break;
            }
            case SIM: {
                kP = simKP;
                kI = simKI;
                kD = simKD;
                break;
            }
            case REPLAY: {
                kP = replayKP;
                kI = replayKI;
                kD = replayKD;
                break;
            }
            default: {
                kP = 0;
                kI = 0;
                kD = 0;
                break;
            }
        }
    }

    /**
     * Constructs a new {@link PIDConstantsAK} with the following parameters.
     *
     * @param realConstants   The {@link PIDConstants} instance used for <code>REAL</code> mode.
     * @param simConstants    The {@link PIDConstants} instance used for <code>SIM</code> mode.
     * @param replayConstants The {@link PIDConstants} instance used for <code>REPLAY</code> mode.
     */
    public PIDConstantsAK(PIDConstants realConstants, PIDConstants simConstants, PIDConstants replayConstants) {
        this(
                realConstants.kP, realConstants.kI, realConstants.kD,
                simConstants.kP, simConstants.kI, simConstants.kD,
                replayConstants.kP, replayConstants.kI, replayConstants.kD
        );
    }

    /**
     * Constructs a new {@link PIDConstantsAK} which only contains <b>one shared set</b> of values.
     * @param kP The P value in <b>ALL MODES</b>
     * @param kI The I value in <b>ALL MODES</b>
     * @param kD The D value in <b>ALL MODES</b>
     */
    public PIDConstantsAK(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /** @return The applicable {@link PIDConstants} based on the {@link OperationMode}. */
    public PIDConstants get() {
        return new PIDConstants(kP, kI, kD);
    }
}
