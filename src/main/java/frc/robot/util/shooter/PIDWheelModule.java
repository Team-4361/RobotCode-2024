package frc.robot.util.shooter;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDConstantsAK;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class PIDWheelModule implements LoggableInputs {
    private final FRCSparkMax motor;

    private final SimpleMotorFeedforward feedFwd;
    private final PIDController controller;
    private final String moduleName;

    private double desiredRPM;

    private double velocityRPM = 0.0;
    private double appliedVolts = 0.0;
    private double currentAmps = 0.0;

    private RelativeEncoder encoder;

    /**
     * Constructs a new {@link PIDWheelModule}.
     * @param motorId    The motor ID
     * @param constants  The {@link PIDConstantsAK} to use.
     * @param moduleName The {@link String} module name
     * @param tuneName   The <b>optional</b> {@link String} tuning name.
     */
    public PIDWheelModule(int motorId, PIDConstantsAK constants, String moduleName, String tuneName) {
        this.motor = new FRCSparkMax(motorId, kBrushless, MotorModel.NEO);
        this.encoder = motor.getEncoder();
        this.controller = PIDConstantsAK.generateController(constants);
        this.moduleName = moduleName;
        this.feedFwd = new SimpleMotorFeedforward(0.1, 0.13, 0);

        if (!tuneName.isBlank())
            IOManager.initPIDTune(tuneName, controller);
    }

    /** @return If the {@link PIDWheelModule} is at target. */
    public boolean atTarget() {
        return ExtendedMath.inToleranceNotZero(desiredRPM, encoder.getVelocity(), 100);
    }

    public void update() {
        encoder = motor.getEncoder();

        velocityRPM = encoder.getVelocity();
        appliedVolts = motor.getAppliedVoltage();
        currentAmps = motor.getOutputCurrent();

        Logger.processInputs(moduleName, this);

        double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
        if (!Constants.isReplay()) {
            motor.setVoltage(
                    feedFwd.calculate(velocityRadPerSec)
                        + controller.calculate(velocityRadPerSec, Units.rotationsPerMinuteToRadiansPerSecond(desiredRPM))
            );
        }

        Logger.recordOutput("Shooter/TargetReached", atTarget());
    }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("VelocityRPM", this.velocityRPM);
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
    }

    /**
     * Sets the target of the {@link PIDWheelModule}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) { this.desiredRPM = rpm; }

    /** Stops the {@link PIDWheelModule} from spinning. */
    public void stop() { setTarget(0); }
}
