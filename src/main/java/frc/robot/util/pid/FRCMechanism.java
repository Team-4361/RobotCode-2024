package frc.robot.util.pid;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.preset.PresetMap;

import java.util.function.Supplier;
import static edu.wpi.first.math.MathUtil.clamp;

/**
 * This {@link FRCMechanism} is intended to make {@link CANSparkMax} PID control easier to
 * implement. It automatically takes care of setting target rotations, encoders, zeroing, etc. This PID system
 * is also designed to be operated manually using the <code>translateMotor</code> method.
 *
 * @author Eric Gold (ericg2)
 */
public class FRCMechanism extends SubsystemBase {
    private final FRCSparkMax motor;
    private final PIDController controller;
    private final String name;
    private final RelativeEncoder encoder;

    private Supplier<Double> presetSupplier;
    private Supplier<Boolean> pidEnabledSupplier;
    private Supplier<Boolean> limitBypassSupplier;

    private double lastTarget = Double.MAX_VALUE;
    private double forwardLimit = Double.MAX_VALUE;
    private double reverseLimit = Double.MIN_VALUE;
    private double targetRotation, maxSpeed, tolerance;
    private boolean teleopMode;

    /**
     * Sets the Target Rotation that the {@link Encoder} should be set to. While teleoperation mode is disabled,
     * the motor will always spin to match the target.
     *
     * @param rotation The amount of rotations to set the Target for.
     * @see #translateMotor(double)
     */
    public void setTarget(double rotation) {
        this.targetRotation = getLimitAdjustedTarget(rotation);
    }

    /**
     * Sets the minimum value this {@link FRCMechanism} is allowed to rotate.
     * @param limit The {@link Double} value to use.
     */
    public void setReverseLimit(double limit) { this.reverseLimit = limit; }

    /**
     * Sets the maximum value this {@link FRCMechanism} is allowed to rotate.
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

    /** @return The {@link Supplier} which can bypass the forward/reverse limits. */
    public Supplier<Boolean> getLimitBypassSupplier() { return this.limitBypassSupplier; }

    public void registerPresets(PresetMap<Double> map) {
        this.presetSupplier = map.getSupplier();
        map.addListener((name, value) -> updateTarget());
    }

    /** @return The current {@link Encoder} position of the {@link CANSparkMax} motor. */
    public double getRotation() { return encoder.getPosition(); }

    public Supplier<Double> getPresetSupplier() { return presetSupplier; }

    /** @return The current Target {@link Encoder} position of the {@link CANSparkMax} motor. */
    public double getTargetRotation() { return targetRotation; }

    /**
     * Manually translates the motor using a given <code>speed</code>. While <code>speed</code> is not zero, the
     * PID control is disabled, allowing manual rotation to occur. The Target Rotation is set to the current {@link Encoder}
     * reading during non-zero operation.
     *
     * @param power A motor power from -1.0 to +1.0 to spin the motor.
     */
    public void translateMotor(double power) {
        if (DriverStation.isTeleop()) {
            if (power == 0 && teleopMode) {
                // Set the target angle to the current rotations to freeze the value and prevent the PIDController from
                // automatically adjusting to the previous value.
                setTarget(getRotation());
                teleopMode = false;
            }
            if (power != 0 && !teleopMode)
                teleopMode = true;

            motor.set(getLimitAdjustedPower(power));
        }
    }

    /**
     * Sets the Tolerance for the {@link PIDController}. This will prevent any encoder inaccuracies from stalling
     * the motor when the target is reached.
     *
     * @param rotations The amount of <b>rotations</b> for Tolerance.
     * @return {@link FRCMechanism}
     */
    public FRCMechanism setTolerance(double rotations) {
        this.tolerance = rotations;
        return this;
    }

    /**
     * Resets the {@link Encoder} used for measuring position.
     */
    public void resetEncoder() {
        encoder.setPosition(0);
        targetRotation = 0;
    }

    /**
     * Sets the Maximum Speed for the {@link PIDController}. This will prevent the system from operating more than
     * plus/minus the specified <code>speed</code>
     *
     * @param speed The <code>speed</code> from -1.0 to +1.0 to use.
     * @return {@link FRCMechanism}
     */
    public FRCMechanism setMaxSpeed(double speed) {
        this.maxSpeed = speed;
        return this;
    }

    private double getLimitAdjustedTarget(double angle) {
        // Do not perform any calculatiosn if the limit bypass supplier is true.
        if (limitBypassSupplier.get()) return angle;

        if (forwardLimit != Double.MAX_VALUE) {
            if (angle > forwardLimit) {
                angle = forwardLimit-0.1;
            }
        }
        if (reverseLimit != Double.MIN_VALUE) {
            if (angle < reverseLimit) {
                angle = reverseLimit+0.1;
            }
        }

        return angle;
    }

    private double getLimitAdjustedPower(double power) {
        if (power == 0) return 0;

        // Do not perform any calculations if the limit bypass supplier is true.
        if (limitBypassSupplier.get()) return power;

        if (forwardLimit != Double.MAX_VALUE) {
            if (power > 0 && encoder.getPosition() >= forwardLimit) {
                return 0;
            } else {
                return power;
            }
        } else if (reverseLimit != Double.MIN_VALUE) {
            if (power < 0 && encoder.getPosition() <= reverseLimit) {
                return 0;
            } else {
                return power;
            }
        } else {
            // At the stage, no limit has been set. Do not perform calculations and return the initial power.
            return power;
        }
    }

    /* The tolerance used for the {@link PIDController}. */
    public double getTolerance() {
        return tolerance;
    }

    /** @return The maximum speed of the {@link PIDController}. */
    public double getMaxSpeed() {
        return maxSpeed;
    }

    public FRCMechanism setPID(double p, double i, double d) {
        controller.setPID(p, i, d);
        return this;
    }

    /**
     * Checks if the actual value is within a specified tolerance of the expected value
     * @param expected The value to be expected.
     * @param actual The actual value.
     * @param tolerance The maximum error or tolerance that the value can be offset to still be true.
     * @return True/false depending on tolerance.
     */
    private static boolean inTolerance(double expected, double actual, double tolerance) {
        return Math.abs(expected - actual) <= tolerance;
    }

    /**
     * @return If the {@link Encoder} is within the tolerance of the Target. Useful for conditions
     * depending on the Motor being moved to a certain position before proceeding.
     */
    public boolean atTarget() {
        return inTolerance(getRotation(), getTargetRotation(), tolerance);
    }

    public FRCMechanism invert(boolean inverted) {
        motor.setInverted(inverted);
        return this;
    }

    public FRCMechanism(String name, FRCSparkMax motor, PIDConstants pidOptions) {
        this.controller = new PIDController(
                pidOptions.kP,
                pidOptions.kI,
                pidOptions.kD
        );

        this.motor = motor;
        this.name = name;
        this.teleopMode = false;
        this.pidEnabledSupplier = () -> true;
        this.limitBypassSupplier = () -> false;
        this.maxSpeed = 1;
        this.tolerance = 0.5;
        this.encoder = motor.getEncoder();
        this.targetRotation = encoder.getPosition();

        controller.setP(pidOptions.kP);
        controller.setI(pidOptions.kI);
        controller.setD(pidOptions.kD);

        motor.enableVoltageCompensation(12);
    }

    public FRCMechanism setPIDControlSupplier(Supplier<Boolean> supplier) {
        pidEnabledSupplier = supplier;
        return this;
    }

    public void updateTarget() {
        setTarget(presetSupplier.get());
        lastTarget = presetSupplier.get();
    }

    public Command resetEncoderCommand() { return this.runOnce(this::resetEncoder); }

    @Override
    public void periodic() {
        if (lastTarget == Double.MAX_VALUE) {
            lastTarget = presetSupplier.get();
        }

        if (!teleopMode && !atTarget() && pidEnabledSupplier.get())
            motor.set(getLimitAdjustedPower(clamp(controller.calculate(getRotation(), getTargetRotation()), -maxSpeed, maxSpeed)));

        SmartDashboard.putNumber(name + " Rotation", getRotation());
        SmartDashboard.putNumber(name + " Target Rotation", getTargetRotation());
        SmartDashboard.putBoolean(name + " At Target", atTarget());

        double suppliedTarget = presetSupplier.get();
        if (lastTarget != presetSupplier.get()) {
            setTarget(suppliedTarget);
            lastTarget = suppliedTarget;
        }
    }
}