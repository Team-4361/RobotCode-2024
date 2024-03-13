package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.preset.PresetMap;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

/**
 * This {@link PIDMechanismBase} class is designed to allow a simple and efficient way to create PID-control
 * mechanisms, with full AdvantageKit logging and replay support.
 *
 * @author Eric Gold
 */
public abstract class PIDMechanismBase {
    private final CANSparkMax motor;
    private final DCMotorSim motorSim;
    private final MotorModel model;
    private boolean simInverted = false;
    private long lastSimUpdateMillis = System.currentTimeMillis();

    private final DashTunablePID pidTune;
    private final PIDController controller;
    private final String moduleName;
    private final boolean rpmControl;

    // All INPUT values are logged here!
    private double targetValue = 0.0;
    private double currentValue = 0.0;
    private double tolerance = 0.0;
    private boolean pidEnabled = true;
    private double forwardLimit = Double.MAX_VALUE;
    private double reverseLimit = Double.MIN_VALUE;
    private double maxSpeed = 1;
    private double lastPower = 0;

    private boolean limitBypassEnabled = false;
    private boolean teleopMode = false;
    private boolean tuningEnabled;

    private Supplier<Boolean> pidEnabledSupplier = () -> true;
    private Supplier<Boolean> limitBypassSupplier = () -> false;

    private final RelativeEncoder encoder;

    public MotorModel getModel() { return this.model; }
    public double getMaxSpeed() { return this.maxSpeed; }

    //region Encoder Bindings
    private RelativeEncoder getEncoder() {
        RelativeEncoder realEncoder = motor.getEncoder();
        return new RelativeEncoder() {
            @Override
            public double getPosition() {
                if (RobotBase.isSimulation() && motorSim != null) {
                    return motorSim.getAngularPositionRotations();
                } else {
                    return realEncoder.getPosition();
                }
            }
            @Override
            public double getVelocity() {
                if (RobotBase.isSimulation() && motorSim != null) {
                    return motorSim.getAngularVelocityRPM();
                } else {
                    return realEncoder.getVelocity();
                }
            }
            @Override
            public REVLibError setPosition(double position) {
                if (RobotBase.isSimulation() && motorSim != null) {
                    motorSim.setState(position, 0);
                    return REVLibError.kOk;
                } else {
                    return realEncoder.setPosition(position);
                }
            }

            @Override
            public REVLibError setPositionConversionFactor(double factor) {
                return realEncoder.setPositionConversionFactor(factor);
            }

            @Override
            public REVLibError setVelocityConversionFactor(double factor) {
                return realEncoder.setVelocityConversionFactor(factor);
            }
            @Override
            public double getPositionConversionFactor() {
                return realEncoder.getPositionConversionFactor();
            }

            @Override
            public double getVelocityConversionFactor() {
                return realEncoder.getVelocityConversionFactor();
            }

            @Override
            public REVLibError setAverageDepth(int depth) {
                return realEncoder.setAverageDepth(depth);
            }

            @Override
            public int getAverageDepth() {
                return realEncoder.getAverageDepth();
            }

            @Override
            public REVLibError setMeasurementPeriod(int period_ms) {
                return realEncoder.setMeasurementPeriod(period_ms);
            }

            @Override
            public int getMeasurementPeriod() {
                return realEncoder.getMeasurementPeriod();
            }

            @Override
            public int getCountsPerRevolution() {
                return realEncoder.getCountsPerRevolution();
            }

            @Override
            public REVLibError setInverted(boolean inverted) {
                if (RobotBase.isSimulation() && motorSim != null) {
                    simInverted = inverted;
                    return REVLibError.kOk;
                } else {
                    return realEncoder.setInverted(inverted);
                }
            }

            @Override
            public boolean getInverted() {
                if (RobotBase.isSimulation() && motorSim != null) {
                    return simInverted;
                }
                return realEncoder.getInverted();
            }
        };
    }
    //endregion

    /**
     * @param motorRotations The motor rotations as reported by the {@link Encoder}.
     *
     * @return The current value which should be used for current positional based control. It MUST be in the
     * <b>SAME UNIT</b> as the Target Position.
     */
    protected abstract double getCurrentPosition(double motorRotations);

    /**
     * Constructs a new {@link PIDMechanismBase}.
     * @param motorId       The motor ID to use.
     * @param constants     The {@link PIDConstants} to use.
     * @param model         The {@link MotorModel} of the {@link CANSparkMax} motor.
     * @param moduleName    The {@link String} module name
     * @param tuningEnabled If PID {@link SmartDashboard} tuning is enabled.
     */
    public PIDMechanismBase(int motorId,
                            PIDConstants constants,
                            MotorModel model,
                            String moduleName,
                            boolean tuningEnabled,
                            boolean rpmControl) {
        this.motor = new CANSparkMax(motorId, kBrushless);
        this.controller = GlobalUtils.generateController(constants);
        this.tuningEnabled = tuningEnabled;
        this.rpmControl = rpmControl;
        this.moduleName = moduleName;
        this.model = model;

        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 20);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 20);
        motor.setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 20);
        motor.enableVoltageCompensation(12.0);

        if (tuningEnabled) {
            pidTune = new DashTunablePID(moduleName + ": PID", constants);
            pidTune.addConsumer(controller::setP, controller::setI, controller::setD);
        } else {
            pidTune = null;
        }

        if (RobotBase.isSimulation()) {
            motorSim = new DCMotorSim(model.getMotorInstance(), 1, 0.025);
            lastSimUpdateMillis = System.currentTimeMillis();
        } else { motorSim = null; }

        this.encoder = getEncoder();
        encoder.setPosition(0);
    }

    /**
     * Sets the maximum speed of this {@link PIDMechanismBase}. Applies to both PID
     * and human control.
     * @param speed The power from 0.0 to +1.0
     */
    public void setMaxPower(double speed) { this.maxSpeed = Math.abs(speed); }

    /** @return The {@link String} name of the {@link PIDMechanismBase}. */
    public String getModuleName() { return this.moduleName; }

    /**
     * Sets the inversion of the underlying {@link CANSparkMax} instance.
     * @param inverted The value to apply.
     */
    public void setInverted(boolean inverted) {
        motor.setInverted(inverted);
    }

    public void registerPresets(PresetMap<Double> map) {
        map.addListener((mapName, value) -> targetValue = value);
    }


    /** @return If the {@link PIDMechanismBase} is at target. */
    public boolean atTarget() {
        return GlobalUtils.inToleranceNotZero(
                targetValue,
                rpmControl ? encoder.getVelocity() : getCurrentPosition(encoder.getPosition()),
                tolerance
        );
    }

    private void setVoltage(double outputVolts) {
        if (RobotBase.isSimulation() && motorSim != null) {
            double simVolts = MathUtil.clamp(outputVolts, -12, 12);
            motorSim.setInputVoltage(simVolts);
        } else {
            motor.setVoltage(outputVolts);
        }
    }

    /** Updates the {@link PIDMechanismBase}. <b>This MUST be called in a periodic/execute method!</b> */
    public void update() {
        if (pidTune != null)
            pidTune.update();

        if (RobotBase.isSimulation()) {
            motorSim.update((System.currentTimeMillis() - lastSimUpdateMillis) / 1000f);
            lastSimUpdateMillis = System.currentTimeMillis();
        }

        double velocityRPM = encoder.getVelocity();
        currentValue = getCurrentPosition(encoder.getPosition());

        // It is required to pull the direct values of these suppliers since AdvantageKit CANNOT log suppliers
        // correctly.
        pidEnabled = pidEnabledSupplier.get();
        limitBypassEnabled = limitBypassSupplier.get();

        if ((targetValue == 0 && rpmControl) || atTarget()) {
            motor.setVoltage(0);
        } else if (pidEnabled && !teleopMode) {
            double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
            if (rpmControl) {
                double targetVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(targetValue);

                setPower(controller.calculate(velocityRadPerSec, targetVelocityRadPerSec));
            } else {
                double positionRad = Units.rotationsToRadians(currentValue);
                double targetPositionRad = Units.rotationsToRadians(targetValue);

                setPower(controller.calculate(positionRad, targetPositionRad));
            }
        }

        if (tuningEnabled) {
            SmartDashboard.putNumber(getModuleName() + " Target", targetValue);
            SmartDashboard.putNumber(getModuleName() + " Tolerance", tolerance);
        }

        SmartDashboard.putNumber(getModuleName() + " Value", currentValue);
    }

    /**
     * Sets the minimum value this {@link PIDMechanismBase} is allowed to rotate.
     * @param limit The {@link Double} value to use.
     */
    public void setReverseLimit(double limit) { this.reverseLimit = limit; }

    /**
     * Sets the maximum value this {@link PIDMechanismBase} is allowed to rotate.
     * @param limit The motor rotation {@link Double} value to use.
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
            return MathUtil.clamp(power, -maxSpeed, maxSpeed); // No limits reached, return original power
        }
    }

    private void setPower(final double speed) {
        double adjustedSpeed = MathUtil.clamp(speed, -maxSpeed, maxSpeed);
        if (RobotBase.isSimulation() && motorSim != null) {
            motorSim.setInputVoltage(adjustedSpeed * 12);
        } else {
            motor.set(adjustedSpeed);
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
        if (rpmControl || !pidEnabled) {
            setPower(getLimitAdjustedPower(power));
        } else if (DriverStation.isTeleop()) {
            if (power == 0 && teleopMode) {
                // Set the target angle to the current rotations to freeze the value and prevent the PIDController from
                // automatically adjusting to the previous value.
                if (lastPower != 0) {
                    setTarget(currentValue);
                    teleopMode = false;
                }
            }
            if (power != 0 && !teleopMode) {
                teleopMode = true;
                setPower(getLimitAdjustedPower(power));
            }
            lastPower = power;
        }
    }

    /**
     * Sets the target of the {@link PIDMechanismBase}.
     * @param value      The value to use.
     */
    public void setTarget(double value) {
       // if (!rpmControl) {
        //    if (forwardLimit != Double.MAX_VALUE) value = Math.min(value, forwardLimit - 0.1); // Enforce forward limit
        //    if (reverseLimit != Double.MIN_VALUE) value = Math.max(value, reverseLimit + 0.1); // Enforce reverse limit
        //}
        this.targetValue = value;
    }

    /** Stops the {@link PIDMechanismBase} from spinning. */
    public void stop() {
        if (!pidEnabled)
            translateMotor(0);
        else
            setTarget(0);
    }

    public void reset() {
        encoder.setPosition(0);
        setTarget(0);
    }
}
