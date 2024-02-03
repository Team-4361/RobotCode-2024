package frc.robot.util.pid;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;
import frc.robot.util.motor.MotorModel;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class PIDWheelModule implements LoggableInputs {
    private final FRCSparkMax motor;

    private final SimpleMotorFeedforward feedFwd;
    private final PIDController controller;
    private final String moduleName;

    private double targetRPM = 0.0;
    private double velocityRPM = 0.0;
    private double appliedVolts = 0.0;
    private double currentAmps = 0.0;

    private RelativeEncoder encoder;

    /**
     * Constructs a new {@link PIDWheelModule}.
     * @param motorId    The motor ID
     * @param constants  The {@link PIDConstantsAK} to use.
     * @param kS         The {@link SimpleMotorFeedforward} kS constant.
     * @param kV         The {@link SimpleMotorFeedforward} kV constant.
     * @param kA         The {@link SimpleMotorFeedforward} kA constant.
     * @param model      The {@link IMotorModel} of the {@link FRCSparkMax} motor.
     * @param moduleName The {@link String} module name
     * @param tuneName   The <b>optional</b> {@link String} tuning name.
     */
    public PIDWheelModule(int motorId,
                          PIDConstantsAK constants,
                          double kS,
                          double kV,
                          double kA,
                          IMotorModel model,
                          String moduleName,
                          String tuneName) {

        this.motor = new FRCSparkMax(motorId, kBrushless, model);
        this.feedFwd = new SimpleMotorFeedforward(kS, kV, kA);
        this.controller = PIDConstantsAK.generateController(constants);

        this.encoder = motor.getEncoder();
        this.moduleName = moduleName;

        if (tuneName != null && !tuneName.isBlank())
            IOManager.initPIDTune(tuneName, controller);
    }

    /** @return If the {@link PIDWheelModule} is at target. */
    public boolean atTarget() {
        return ExtendedMath.inToleranceNotZero(targetRPM, encoder.getVelocity(), 100);
    }

    public void update() {
        encoder = motor.getEncoder();

        velocityRPM = encoder.getVelocity();
        appliedVolts = motor.getAppliedVoltage();
        currentAmps = motor.getOutputCurrent();

        Logger.processInputs(moduleName, this);

        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        if (!Constants.isReplay()) {
            if (targetRPM == 0) {
                motor.setVoltage(0);
            } else {
                motor.setVoltage(
                        feedFwd.calculate(velocityRadPerSec)
                                + controller.calculate(velocityRadPerSec, Units.rotationsPerMinuteToRadiansPerSecond(targetRPM))
                );
            }
        }

        Logger.recordOutput(moduleName + "/TargetReached", atTarget());
    }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("VelocityRPM", this.velocityRPM);
        table.put("TargetRPM", this.targetRPM);
        table.put("AppliedVolts", this.appliedVolts);
        table.put("CurrentAmps", this.currentAmps);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.velocityRPM =  table.get("VelocityRPM", this.velocityRPM);
        this.appliedVolts = table.get("AppliedVolts", this.appliedVolts);
        this.currentAmps =  table.get("CurrentAmps", this.currentAmps);
        this.targetRPM   = table.get("TargetRPM", this.targetRPM);
    }

    /**
     * Sets the target of the {@link PIDWheelModule}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) { this.targetRPM = rpm; }

    /** Stops the {@link PIDWheelModule} from spinning. */
    public void stop() { setTarget(0); }
}
