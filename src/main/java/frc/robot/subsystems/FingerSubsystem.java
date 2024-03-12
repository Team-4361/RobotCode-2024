package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDLinearMechanism;
import frc.robot.util.pid.PIDLinearMechanism.DistanceUnit;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;
import frc.robot.util.preset.PresetMap;

import static frc.robot.Constants.Debug.TRAP_ARM_TUNING_ENABLED;
import static frc.robot.Constants.TrapArm.*;

/**
 * This {@link FingerSubsystem} is designed to control the {@link Robot}'s arm. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for extending.
 */
public class FingerSubsystem extends SubsystemBase {
    private final PIDRotationalMechanism extensionMechanism;

    /** Constructs a new {@link PIDRotationalMechanism}. */
    public FingerSubsystem() {
        this.extensionMechanism = new PIDRotationalMechanism(
                ARM_EXTENSION_MOTOR_ID,
                ARM_EXTENSION_PID,
                MotorModel.NEO,
                "Arm",
                TRAP_ARM_TUNING_ENABLED,
                //ARM_GEAR_RATIO,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                false
        );

        //extensionMechanism.setDistanceTuningEnabled(TRAP_ARM_TUNING_ENABLED);
        extensionMechanism.setInverted(true);
        extensionMechanism.setForwardLimit(ARM_MAX_ROTATION);
    }

    @Override
    public void periodic() {
        extensionMechanism.update();
    }

    public void registerExtensionPresets(PresetMap<Double> map) {
        extensionMechanism.registerPresets(map);
    }

    /**
     * Sets the target linear {@link Servo} position, changing the angle of the {@link FingerSubsystem}.
     */
    public void setAnglePosition(double rotations) { extensionMechanism.setTarget(rotations); }
    public void reset() { extensionMechanism.reset(); }

    /**
     * Sets the extension speed of the {@link FingerSubsystem}.
     * @param speed The {@link Double} value from -1.0 to +1.0
     */
    public void setExtensionSpeed(double speed) { extensionMechanism.translateMotor(speed); }
}