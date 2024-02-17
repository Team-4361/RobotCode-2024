package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.Robot;
import swervelib.parser.PIDFConfig;

import java.util.function.Consumer;

/**
 * This {@link PIDConstantsAK} class is designed to easily switch PID values with various modes.
 */
public class PIDConstantsAK {
    public final double kP;
    public final double kI;
    public final double kD;

    /**
     * Generates a {@link PIDController} based upon the input {@link PIDConstantsAK}.
     * @param constants The {@link PIDConstantsAK} to use.
     * @return The created {@link PIDController} instance.
     */
    public static PIDController generateController(PIDConstantsAK constants) {
        return new PIDController(constants.kP, constants.kI, constants.kD);
    }

    public PIDFConfig toPIDF() {
        return new PIDFConfig(
                kP, kI, kD
        );
    }

    public void initController(Consumer<Double> p, Consumer<Double> i, Consumer<Double> d) {
        p.accept(kP);
        i.accept(kI);
        d.accept(kD);
    }

    public void initController(PIDController... controllers) {
        for (PIDController controller : controllers) {
            initController(controller::setP, controller::setI, controller::setD);
        }
    }

    public void initController(SparkPIDController... controllers) {
        for (SparkPIDController controller : controllers) {
            initController(controller::setP, controller::setI, controller::setD);
        }
    }

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
     */
    public PIDConstantsAK(
            double realKP,   double realKI,   double realKD,
            double simKP,    double simKI,    double simKD
    ) {
        kP = RobotBase.isSimulation() ? simKP : realKP;
        kI = RobotBase.isSimulation() ? simKI : realKI;
        kD = RobotBase.isSimulation() ? simKD : realKD;
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

    /** @return The applicable {@link PIDConstants} based on the {@link Robot}. */
    public PIDConstants get() { return new PIDConstants(kP, kI, kD); }
}
