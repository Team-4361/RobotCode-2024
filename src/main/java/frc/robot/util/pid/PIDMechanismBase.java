package frc.robot.util.pid;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;
import frc.robot.util.preset.PresetMap;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

/**
 * This {@link PIDMechanismBase} class is designed to allow a simple and efficient way to create PID-control
 * mechanisms, with full AdvantageKit logging and replay support.
 *
 * @author Eric Gold
 */
public abstract class PIDMechanismBase extends PresetMap<Double> implements LoggableInputs {
    private final FRCSparkMax motor;

    private final SimpleMotorFeedforward feedFwd;
    private final PIDController controller;
    private final String moduleName;

    // All INPUT values are logged here!
    private double targetValue = 0.0;
    private boolean rpmControl = false;
    private double velocityRPM = 0.0;
    private double appliedVolts = 0.0;
    private double currentAmps = 0.0;
    private double currentPosition = 0.0;
    private boolean pidEnabled = true;
    private boolean limitBypassEnabled = false;
    private double tolerance = 0.0;
    private boolean teleopMode = true;
    private double forwardLimit = Double.MAX_VALUE;
    private double reverseLimit = Double.MIN_VALUE;

    private Supplier<Boolean> pidEnabledSupplier;
    private Supplier<Boolean> limitBypassSupplier;

    private RelativeEncoder encoder;

    /**
     * @param motorRotations The motor rotations as reported by the {@link Encoder}.
     *
     * @return The current value which should be used for current positional based control. It MUST be in the
     * <b>SAME UNIT</b> as the Target Position.
     */
    protected abstract double getCurrentPosition(double motorRotations);

    /**
     * Constructs a new {@link PIDMechanismBase}.
     * @param motorId    The motor ID
     * @param constants  The {@link PIDConstantsAK} to use.
     * @param kS         The {@link SimpleMotorFeedforward} kS constant.
     * @param kV         The {@link SimpleMotorFeedforward} kV constant.
     * @param kA         The {@link SimpleMotorFeedforward} kA constant.
     * @param model      The {@link IMotorModel} of the {@link FRCSparkMax} motor.
     * @param moduleName The {@link String} module name
     * @param tuneName   The <b>optional</b> {@link String} tuning name.
     */
    public PIDMechanismBase(int motorId,
                            PIDConstantsAK constants,
                            double kS,
                            double kV,
                            double kA,
                            IMotorModel model,
                            String moduleName,
                            String tuneName) {

        super(moduleName, tuneName != null && !tuneName.isBlank());

        this.motor = new FRCSparkMax(motorId, kBrushless, model);
        this.feedFwd = new SimpleMotorFeedforward(kS, kV, kA);
        this.controller = PIDConstantsAK.generateController(constants);

        this.encoder = motor.getEncoder();
        this.moduleName = moduleName;

        if (tuneName != null && !tuneName.isBlank())
            IOManager.initPIDTune(tuneName, controller);

        addListener((name, value) -> setTarget(value));
    }

    /** @return If the {@link PIDMechanismBase} is at target. */
    public boolean atTarget() {
        return ExtendedMath.inToleranceNotZero(
                targetValue,
                rpmControl ? encoder.getVelocity() : getCurrentPosition(encoder.getPosition()),
                tolerance
        );
    }

    /** Updates the {@link PIDMechanismBase}. <b>This MUST be called in a periodic/execute method!</b> */
    public void update() {
        encoder = motor.getEncoder();

        velocityRPM = encoder.getVelocity();
        currentPosition = getCurrentPosition(encoder.getPosition());
        appliedVolts = motor.getAppliedVoltage();
        currentAmps = motor.getOutputCurrent();

        // It is required to pull the direct values of these suppliers since AdvantageKit CANNOT log suppliers
        // correctly.
        pidEnabled = pidEnabledSupplier.get();
        limitBypassEnabled = limitBypassSupplier.get();

        Logger.processInputs(moduleName, this);

        if ((targetValue == 0 && rpmControl) || atTarget()) {
            motor.setVoltage(0);
        } else if (!Constants.isReplay() && pidEnabled && !teleopMode) {
            double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
            if (rpmControl) {
                double targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetValue);

                motor.setVoltage(
                        feedFwd.calculate(velocityRadPerSec)
                            + controller.calculate(velocityRadPerSec, targetVelocityRadPerSec)
                );
            } else {
                double positionRad = Units.rotationsToRadians(currentPosition);
                double targetPositionRad = Units.rotationsToRadians(targetValue);

                motor.setVoltage(
                        feedFwd.calculate(velocityRadPerSec)
                            + controller.calculate(positionRad, targetPositionRad)
                );
            }
        }

        Logger.recordOutput(moduleName + "/TargetReached", atTarget());
    }

    /**
     * Sets the minimum value this {@link PIDMechanismBase} is allowed to rotate.
     * @param limit The {@link Double} value to use.
     */
    public void setReverseLimit(double limit) { this.reverseLimit = limit; }

    /**
     * Sets the maximum value this {@link PIDMechanismBase} is allowed to rotate.
     * @param limit The {@link Double} value to use.
     */
    public void setForwardLimit(double limit) { this.forwardLimit = limit; }

    /**
     * Sets a {@link Supplier} which can bypass the forward/reverse limits.
     * @param supplier The {@link Boolean} {@link Supplier} to use.
     */
    public void setLimitBypassSupplier(Supplier<Boolean> supplier) { this.limitBypassSupplier = supplier; }

    /** @return The current reverse limit, or {@link Double#MIN_VALUE} if there is none. */
    public double getReverseLimit() { return this.reverseLimit; }

    /** @return The current forward limit, or {@link Double#MAX_VALUE} if there is none. */
    public double getForwardLimit() { return this.forwardLimit; }

    /** @return The tolerance used for the {@link PIDController}. */
    public double getTolerance() { return tolerance; }

    /** @return The {@link Supplier} which can bypass the forward/reverse limits. */
    public Supplier<Boolean> getLimitBypassSupplier() { return this.limitBypassSupplier; }

    /**
     * Calculates the limit-switch adjusted Power (-1 to +1) OR Voltage (-12 to +12) based on the inputs.
     * @param power The power OR voltage to use.
     * @return The adjusted {@link Double} value.
     */
    private double getLimitAdjustedPower(double power) {
        if (power == 0 || limitBypassEnabled) {
            // No need for adjustments if power is zero or limits are bypassed
            return power;
        }

        // Check for forward or reverse limit violations and adjust power accordingly
        if (forwardLimit != Double.MAX_VALUE && power > 0 && encoder.getPosition() >= forwardLimit) {
            return 0; // Stop at forward limit
        } else if (reverseLimit != Double.MIN_VALUE && power < 0 && encoder.getPosition() <= reverseLimit) {
            return 0; // Stop at reverse limit
        } else {
            return power; // No limits reached, return original power
        }
    }

    /**
     * Sets the Tolerance for the {@link PIDController}. This will prevent any encoder inaccuracies from stalling
     * the motor when the target is reached.
     *
     * @param rotations The amount of <b>rotations</b> for Tolerance.
     */
    public void setTolerance(double rotations) { this.tolerance = rotations; }

    /**
     * Sets the {@link Boolean} {@link Supplier} which is responsible for determining if PID-control should be enabled
     * for this {@link PIDMechanismBase}.
     * @param supplier The {@link Boolean} {@link Supplier} to use.
     */
    public void setPIDControlSupplier(Supplier<Boolean> supplier) { pidEnabledSupplier = supplier; }

    /**
     * Manually translates the motor using a given <code>speed</code>. While <code>speed</code> is not zero, the
     * PID control is disabled, allowing manual rotation to occur. The Target Rotation is set to the current
     * {@link Encoder} reading during non-zero operation.
     *
     * @param power A motor power from -1.0 to +1.0 to spin the motor.
     */
    public void translateMotor(double power) {
        if (DriverStation.isTeleop()) {
            if (power == 0 && teleopMode) {
                // Set the target angle to the current rotations to freeze the value and prevent the PIDController from
                // automatically adjusting to the previous value.
                setTarget(currentPosition, false);
                teleopMode = false;
            }
            if (power != 0 && !teleopMode)
                teleopMode = true;

            motor.set(getLimitAdjustedPower(power));
        }
    }

    /**
     * Updates a LogTable with the data to log.
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("TargetValue", this.targetValue);
        table.put("RPMControl", this.rpmControl);
        table.put("VelocityRPM", this.velocityRPM);
        table.put("AppliedVolts", this.appliedVolts);
        table.put("CurrentAmps", this.currentAmps);
        table.put("CurrentValue", this.currentPosition);
        table.put("PIDEnabled", this.pidEnabled);
        table.put("LimitBypassed", this.limitBypassEnabled);
        table.put("ValueTolerance", this.tolerance);
        table.put("ManualControl", this.teleopMode);
        table.put("ForwardLimit", this.forwardLimit);
        table.put("ReverseLimit", this.reverseLimit);
    }

    /**
     * Updates data based on a LogTable.
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.targetValue         = table.get("TargetValue", this.targetValue);
        this.rpmControl          = table.get("RPMControl", this.rpmControl);
        this.velocityRPM         = table.get("VelocityRPM", this.velocityRPM);
        this.appliedVolts        = table.get("AppliedVolts", this.appliedVolts);
        this.currentAmps         = table.get("CurrentAmps", this.currentAmps);
        this.currentPosition     = table.get("CurrentValue", this.currentPosition);
        this.pidEnabled          = table.get("PIDEnabled", this.pidEnabled);
        this.limitBypassEnabled  = table.get("LimitBypassed", this.limitBypassEnabled);
        this.tolerance           = table.get("ValueTolerance", this.tolerance);
        this.teleopMode          = table.get("ManualControl", this.teleopMode);
        this.forwardLimit        = table.get("ForwardLimit", this.forwardLimit);
        this.reverseLimit        = table.get("ReverseLimit", this.reverseLimit);
    }

    /**
     * Sets the target of the {@link PIDMechanismBase}.
     * @param value      The value to use.
     * @param rpmControl If the value indicates RPM or position.
     */
    public void setTarget(double value, boolean rpmControl) {
        if (!rpmControl) {
            if (forwardLimit != Double.MAX_VALUE) value = Math.min(value, forwardLimit - 0.1); // Enforce forward limit
            if (reverseLimit != Double.MIN_VALUE) value = Math.max(value, reverseLimit + 0.1); // Enforce reverse limit
        }
        if (rpmControl != this.rpmControl)
            controller.reset(); // reset the controller when switching between position and RPM control.

        this.rpmControl = rpmControl;
        this.targetValue = value;
    }

    /**
     * Sets the target of the {@link PIDMechanismBase} while retaining the current RPM/Position mode.
     * @param value The value to use.
     */
    public void setTarget(double value) { setTarget(value, this.rpmControl); }

    /** Stops the {@link PIDMechanismBase} from spinning. */
    public void stop() { setTarget(0, true); }
}