package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.motor.MotorServo;
import frc.robot.util.pid.PIDLinearMechanism;
import frc.robot.util.pid.PIDLinearMechanism.DistanceUnit;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.preset.PresetMap;

import static frc.robot.Constants.Debug.TRAP_ARM_TUNING_ENABLED;
import static frc.robot.Constants.TrapArm.*;

/**
 * This {@link TrapArmSubsystem} is designed to control the {@link Robot}'s arm. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for extending.
 */
public class TrapArmSubsystem extends SubsystemBase {
    private final MotorServo linearServo;
    private final PIDLinearMechanism mechanism;

    public double getAngleDistanceMM() { return linearServo.getDistanceMM(); }
    public double getAngleTargetMM() { return linearServo.getTargetMM(); }

    /** Constructs a new {@link PIDRotationalMechanism}. */
    public TrapArmSubsystem() {
        this.mechanism = new PIDLinearMechanism(
                ARM_MOTOR_ID,
                ARM_PID,
                ARM_KS,
                ARM_KV,
                ARM_KA,
                MotorModel.NEO_550,
                "Arm",
                TRAP_ARM_TUNING_ENABLED,
                DistanceUnit.INCHES,
                ARM_DISTANCE
        );

        linearServo = new MotorServo(
                ARM_SERVO_ID,
                ARM_MAX_US,
                ARM_DEAD_BAND_MAX_US,
                ARM_CENTER_US,
                ARM_DEAD_BAND_MIN_US,
                ARM_MIN_US,
                ARM_SERVO_MIN_MM,
                ARM_SERVO_MAX_MM
        );

        //mechanism.setDistanceTuningEnabled(TRAP_ARM_TUNING_ENABLED);
        mechanism.setDashboardEnabled(TRAP_ARM_TUNING_ENABLED);
    }

    @Override
    public void periodic() {
        mechanism.update();
        linearServo.update();
        if (TRAP_ARM_TUNING_ENABLED) {
            SmartDashboard.putNumber("Arm: ROT Pos", getAngleTargetMM());
        }
    }

    public void registerAnglePresets(PresetMap<Double> map) {
        map.addListener((mapName, value) -> setAnglePosition(value));
    }
    public void registerExtensionPresets(PresetMap<Double> map) {
        mechanism.registerPresets(map);
    }

    /**
     * Sets the target linear {@link Servo} position, changing the angle of the {@link TrapArmSubsystem}.
     * @param mm The {@link Double} value in millimeters.
     */
    public void setAnglePosition(double mm) {
        linearServo.setDistance(mm);
    }

    public void reset() {
        mechanism.reset();
    }

    /**
     * Sets the extension speed of the {@link TrapArmSubsystem}.
     * @param speed The {@link Double} value from -1.0 to +1.0
     */
    public void setExtensionSpeed(double speed) {
        mechanism.translateMotor(speed);
    }

    /**
     * Sets the speed of the linear {@link Servo}.
     * @param speed The {@link Double} value from -1.0 to +1.0
     */
    public void setAngleSpeed(double speed) {
        linearServo.set(speed);
    }
}