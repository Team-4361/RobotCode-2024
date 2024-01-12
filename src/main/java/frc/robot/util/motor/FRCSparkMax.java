package frc.robot.util.motor;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.jni.CANSparkMaxJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Temperature;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import frc.robot.Constants;
import frc.robot.util.io.Alert;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.GearRatio;
import swervelib.math.SwerveMath;
import swervelib.parser.SwerveModuleConfiguration;

import static edu.wpi.first.units.Units.Celsius;
import static frc.robot.Constants.AlertConfig.STRING_MOTOR_OVER_TEMP;
import static frc.robot.Constants.LooperConfig.STRING_PERIODIC_NAME;

/**
 * This class enables a safe interaction with {@link CANSparkMax} motors; temperature control and watchdogs
 * are both used to prevent dangerous situations.
 *
 * @author Eric Gold
 * @since 0.0.0
 * @version 0.0.1
 */
public class FRCSparkMax extends CANSparkMax {
    private final Alert conditionAlert;
    private final DCMotor model;
    private long lastSimUpdateMillis;

    /**
     * Create a new object to control a SPARK MAX motor Controller
     *
     * @param deviceId The device ID.
     * @param type     The motor type connected to the controller. Brushless motor wires must be connected to their
     *                 matching colors and the hall sensor must be plugged in. Brushed motors must be connected to the
     *                 Red and Black terminals only.
     * @param model    The {@link DCMotor} module which best represents the {@link FRCSparkMax}
     */
    public FRCSparkMax(int deviceId, MotorType type, DCMotor model) {
        super(deviceId, type);

        conditionAlert = IOManager.getAlert(STRING_MOTOR_OVER_TEMP.replace("%ID%", String.valueOf(deviceId)), AlertType.ERROR)
                .setEnableDelay(2000)
                .setDisableDelay(2000)
                .setPersistence(false)
                .setOneUse(false);

        if (type == MotorType.kBrushed) {
            IOManager.warn(this, "Motor #" + deviceId + " is brushed. No stall detection allowed.");
            conditionAlert.setCondition(() -> getMotorTemperature() >= 60);
            return;
        }

        // The motor is brushless; use the encoder to detect velocity for stall detection.
        conditionAlert.setCondition(() -> getMotorTemperature() >= 60 ||
                (getOutputCurrent() >= model.stallCurrentAmps-20 && getEncoder().getVelocity() <= 10)
        );

        if (RobotBase.isSimulation()) {
            IOManager.warnOnFail(
                    setSimFreeSpeed(Units.radiansPerSecondToRotationsPerMinute(model.freeSpeedRadPerSec))
            );
            IOManager.warnOnFail(setSimStallTorque(model.stallTorqueNewtonMeters));

            IOManager.debug(this, "Adding SparkMax ID #" + deviceId + " to simulation.");
            lastSimUpdateMillis = System.currentTimeMillis();
            IOManager
                    .getLoop(STRING_PERIODIC_NAME)
                    .addPeriodic(() -> {
                        final RelativeEncoder relativeEncoder = getEncoder();
                        final double position = relativeEncoder.getPosition();
                        final double velocity = relativeEncoder.getVelocity();
                        final double positionConversionFactor = relativeEncoder.getPositionConversionFactor();

                        relativeEncoder.setPosition(
                                position + velocity *
                                        (System.currentTimeMillis() - lastSimUpdateMillis) / 60000.0
                                        * positionConversionFactor
                        );
                        lastSimUpdateMillis = System.currentTimeMillis();
                    });
        }
    }

    /** @return The {@link DCMotor} model used for the {@link FRCSparkMax}. */
    public DCMotor getModel() { return this.model; }

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
     * Get if the internal SparkMAX has been closed.
     * @return true if already closed, false if not.
     * @see com.revrobotics.CANSparkMaxLowLevel#isClosed
     */
    public boolean isClosed() {
        return isClosed.get();
    }

    /**
     * Internal Rev method, made public through overriding {@link CANSparkMaxLowLevel#throwIfClosed()}
     */
    @Override
    public void throwIfClosed() {
        super.throwIfClosed();
    }

    @Override
    public void set(final double speed) {
        if (conditionAlert.isEnabled())
            super.set(0);
        else
            super.set(MathUtil.clamp(speed, -1, 1));
    }

    public void set(final ControlType controlType, final double value) {
        if (conditionAlert.isEnabled())
            this.getPIDController().setReference(0, controlType);
        else
            this.getPIDController().setReference(value, controlType);
    }
}
