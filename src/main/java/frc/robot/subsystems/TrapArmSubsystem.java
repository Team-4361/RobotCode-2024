package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDLinearMechanism;
import frc.robot.util.pid.PIDLinearMechanism.DistanceUnit;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.preset.PresetMap;

import static frc.robot.Constants.Debug.TRAP_ARM_TUNING_ENABLED;
import static frc.robot.Constants.Debug.TRAP_ROTATION_TUNING_ENABLED;
import static frc.robot.Constants.TrapArm.*;
import static frc.robot.util.pid.PIDRotationalMechanism.RotationUnit.ROTATIONS;

/**
 * This {@link TrapArmSubsystem} is designed to control the {@link Robot}'s arm. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for extending.
 */
public class TrapArmSubsystem extends SubsystemBase {
    private final PIDRotationalMechanism rotationalMechanism;
    private final PIDLinearMechanism extensionMechanism;

    /*
    public double getAngleDistanceMM() { return linearServo.getDistanceMM(); }
    public double getAngleTargetMM() { return linearServo.getTargetMM(); }
     */

    /** Constructs a new {@link PIDRotationalMechanism}. */
    public TrapArmSubsystem() {
        this.extensionMechanism = new PIDLinearMechanism(
                ARM_EXTENSION_MOTOR_ID,
                ARM_EXTENSION_PID,
                ARM_EXTENSION_KS,
                ARM_EXTENSION_KV,
                ARM_EXTENSION_KA,
                MotorModel.NEO_550,
                "Arm",
                TRAP_ARM_TUNING_ENABLED,
                DistanceUnit.INCHES,
                ARM_DISTANCE
        );

        this.rotationalMechanism = new PIDRotationalMechanism(
                ARM_ROTATION_MOTOR_ID,
                ARM_ROTATION_PID,
                ARM_ROTATION_KS,
                ARM_ROTATION_KV,
                ARM_ROTATION_KA,
                MotorModel.NEO_550,
                "Arm Rotation",
                TRAP_ROTATION_TUNING_ENABLED,
                ARM_ROTATION_GEAR_RATIO,
                ROTATIONS,
                false
        );

        //extensionMechanism.setDistanceTuningEnabled(TRAP_ARM_TUNING_ENABLED);
        extensionMechanism.setDashboardEnabled(true);
        //rotationalMechanism.setReverseLimit(ARM_MAX_ROTATION);
        rotationalMechanism.setInverted(true);
    }

    @Override
    public void periodic() {
        extensionMechanism.update();
        rotationalMechanism.update();
        /*
        if (TRAP_ARM_TUNING_ENABLED) {
            SmartDashboard.putNumber("Arm: ROT Pos", getAngleTargetMM());
        }
         */
    }

    public void registerAnglePresets(PresetMap<Double> map) {
        //map.addListener((mapName, value) -> setAnglePosition(value));
        rotationalMechanism.registerPresets(map);
    }
    public void registerExtensionPresets(PresetMap<Double> map) {
        extensionMechanism.registerPresets(map);
    }

    /**
     * Sets the target linear {@link Servo} position, changing the angle of the {@link TrapArmSubsystem}.
     */
    public void setAnglePosition(double rotations) { rotationalMechanism.setTarget(rotations); }

    public void reset() { extensionMechanism.reset(); rotationalMechanism.reset(); }

    /**
     * Sets the extension speed of the {@link TrapArmSubsystem}.
     * @param speed The {@link Double} value from -1.0 to +1.0
     */
    public void setExtensionSpeed(double speed) { extensionMechanism.translateMotor(speed); }

    /**
     * Sets the speed of the linear {@link Servo}.
     * @param speed The {@link Double} value from -1.0 to +1.0
     */
    public void setAngleSpeed(double speed) { rotationalMechanism.translateMotor(speed); }
}