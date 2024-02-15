package frc.robot.util.motor;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;
import frc.robot.util.io.Alert;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.GearRatio;

import static frc.robot.Constants.AlertConfig.STRING_MOTOR_OVER_TEMP;

/**
 * This class enables a safe interaction with {@link CANSparkMax} motors; temperature control and watchdogs
 * are both used to prevent dangerous situations.
 *
 * @author Eric Gold
 * @since 0.0.0
 * @version 0.0.1
 */
public class FRCSparkMax extends CANSparkMax implements IMotorModel {
    private final Alert conditionAlert;
    private final IMotorModel model;
    private final DCMotorSim motorSim;
    private final GearRatio ratio;
    private boolean simInverted = false;
    private double simVolts = 0.0;

    /**
     * @return The motor controller's output current in Amps.
     */
    @Override
    public double getOutputCurrent() {
        if (RobotBase.isSimulation() && motorSim != null) {
            return motorSim.getCurrentDrawAmps();
        } else {
            return super.getOutputCurrent();
        }
    }

    /**
     * Returns an object for interfacing with the hall sensor integrated into a brushless motor, which
     * is connected to the front port of the SPARK MAX.
     *
     * <p>To access a quadrature encoder connected to the encoder pins or the front port of the SPARK
     * MAX, you must call the version of this method with EncoderType and countsPerRev parameters.
     *
     * @return An object for interfacing with the integrated encoder.
     */
    @Override
    public RelativeEncoder getEncoder() {
        RelativeEncoder realEncoder = super.getEncoder();
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
                    motorSim.setState(0, 0);
                    return REVLibError.kOk;
                } else {
                    return realEncoder.setPosition(0);
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

    private long lastSimUpdateMillis;

    /** @return The {@link GearRatio} used for simulation. */
    public GearRatio getRatio() { return this.ratio; }

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected to their
     *                 matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the
     *                 Red and Black terminals only.
     * @param model    The {@link DCMotor} module which best represents the {@link FRCSparkMax}
     * @param ratio    The {@link GearRatio} of the {@link FRCSparkMax}.
     */
    public FRCSparkMax(int deviceId, MotorType type, IMotorModel model, GearRatio ratio) {
        super(deviceId, type);
        this.model = model;
        this.ratio = ratio;

        conditionAlert = IOManager.getAlert(STRING_MOTOR_OVER_TEMP.replace("%ID%", String.valueOf(deviceId)), AlertType.ERROR)
                .setEnableDelay(2000)
                .setDisableDelay(2000)
                .setPersistent(false)
                .setOneUse(false);

        if (type == MotorType.kBrushed) {
            IOManager.warn(this, "Motor #" + deviceId + " is brushed. No stall detection allowed.");
            conditionAlert.setCondition(() -> getMotorTemperature() >= 60);
            this.motorSim = null;
            return;
        }

        // Tweaks to allow the Motor to work better with PID control.
        //restoreFactoryDefaults();
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus1, 500);
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus2, 500);
        setPeriodicFramePeriod(CANSparkLowLevel.PeriodicFrame.kStatus3, 500);
        enableVoltageCompensation(12.0);

        // The motor is brushless; use the encoder to detect velocity for stall detection
        if (!RobotBase.isSimulation()) {
            conditionAlert.setCondition(() -> getMotorTemperature() >= 60 ||
                    (getOutputCurrent() >= model.getMaximumStallCurrent() - 20 && getEncoder().getVelocity() <= 10)
            );
        }

        if (RobotBase.isSimulation()) {
            IOManager.warnOnFail(setSimFreeSpeed(model.getFreeSpeedRPM()));
            IOManager.warnOnFail(setSimStallTorque(model.getStallTorqueNM()));

            motorSim = new DCMotorSim(model.getMotorInstance(1), ratio.getDivisor(), 0.025);
            lastSimUpdateMillis = System.currentTimeMillis();
        } else { motorSim = null; }
    }

    public void updateSim() {
        if (RobotBase.isSimulation()) {
            // Update the DCMotorSim instance approximately every 20ms.
            motorSim.update((System.currentTimeMillis() - lastSimUpdateMillis) / 1000f);
            lastSimUpdateMillis = System.currentTimeMillis();
        }
    }

    public boolean isStalling() { return conditionAlert.isEnabled(); }

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected to their
     *                 matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the
     *                 Red and Black terminals only.
     * @param model    The {@link DCMotor} module which best represents the {@link FRCSparkMax}
     */
    public FRCSparkMax(int deviceId, MotorType type, IMotorModel model) {
        this(deviceId, type, model, GearRatio.DIRECT_DRIVE);
    }

    /**
     * Set the free speed of the motor being simulated.
     *
     * @param freeSpeed the free speed (RPM) of the motor connected to spark max
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSimFreeSpeed(final double freeSpeed) {
        throwIfClosed();
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetSimFreeSpeed(sparkMaxHandle, (float)freeSpeed));
    }

    /**
     * Set the stall torque of the motor being simulated.
     *
     * @param stallTorque The stall torque (N m) of the motor connected to sparkmax
     * @return {@link REVLibError#kOk} if successful
     */
    public REVLibError setSimStallTorque(final double stallTorque) {
        throwIfClosed();
        return REVLibError.fromInt(
                CANSparkMaxJNI.c_SparkMax_SetSimStallTorque(sparkMaxHandle, (float) stallTorque));
    }
    /**
     * Internal Rev method, made public through overriding.
     */
    @Override public void throwIfClosed() { super.throwIfClosed(); }

    @Override
    public void set(final double speed) {
        double adjustedSpeed = MathUtil.clamp(speed, -1, 1);
        if (conditionAlert.isEnabled())
            super.set(0);
        else if (RobotBase.isSimulation() && motorSim != null) {
            motorSim.setInputVoltage(adjustedSpeed * 12);
        } else {
            super.set(adjustedSpeed);
        }
    }

    /**
     * Sets the voltage output of the SpeedController. This is equivalent to a call to
     * SetReference(output, rev::ControlType::kVoltage). The behavior of this call differs slightly
     * from the WPILib documentation for this call since the device internally sets the desired voltage
     * (not a compensation value). That means that this *can* be a 'set-and-forget' call.
     *
     * @param outputVolts The voltage to output.
     */
    @Override
    public void setVoltage(double outputVolts) {
        if (RobotBase.isSimulation() && motorSim != null) {
            simVolts = MathUtil.clamp(outputVolts, -12, 12);
            motorSim.setInputVoltage(simVolts);
        } else {
            super.setVoltage(outputVolts);
        }
    }

    public double getAppliedVoltage() {
        if (RobotBase.isSimulation())
            return simVolts;
        else
            return getAppliedOutput() * getBusVoltage();
    }

    public void set(final ControlType controlType, final double value) {
        if (conditionAlert.isEnabled())
            this.getPIDController().setReference(0, controlType);
        else
            this.getPIDController().setReference(value, controlType);
    }

    /** @return The maximum <b>theoretical</b> stall current of this {@link IMotorModel} in <b>amperes.</b> */
    @Override public int getMaximumStallCurrent() { return model.getMaximumStallCurrent(); }

    /** @return The maximum <b>theoretical</b> free speed of this {@link IMotorModel} in <b>RPM.</b> */
    @Override public double getFreeSpeedRPM() { return model.getFreeSpeedRPM(); }

    /** @return The maximum <b>theoretical</b> stall torque of this {@link IMotorModel} in <b>newton-meters.</b> */
    @Override public double getStallTorqueNM() { return model.getStallTorqueNM(); }

    /**
     * @param numMotors The number of motors to process.
     * @return The {@link DCMotor} instance used for simulation.
     */
    @Override public DCMotor getMotorInstance(int numMotors) { return model.getMotorInstance(numMotors); }
}
