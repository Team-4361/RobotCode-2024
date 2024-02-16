package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDLinearMechanism;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDLinearMechanism.DistanceUnit;
import frc.robot.util.preset.IPresetContainer;
import frc.robot.util.preset.PresetGroup;
import frc.robot.util.preset.PresetMap;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Debug.TRAP_ARM_TUNING_ENABLED;

/**
 * This {@link TrapArmSubsystem} is designed to control the {@link Robot}'s arm. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for extending.
 */
public class TrapArmSubsystem extends SubsystemBase {
    private final Servo linearServo;
    private final PIDLinearMechanism mechanism;
    public double extensionPosition = 0.0;
    public double extensionTarget = 0.0;

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
                TRAP_ARM_TUNING_ENABLED ? "Arm: PID" : "",
                DistanceUnit.INCHES,
                ARM_DISTANCE
        );
        this.linearServo = new Servo(ARM_SERVO_ID);
        linearServo.setBoundsMicroseconds(
                ARM_MAX_US,
                ARM_DEAD_BAND_MAX_US,
                ARM_CENTER_US,
                ARM_DEAD_BAND_MIN_US,
                ARM_MIN_US
        );
        mechanism.setTuneMode(TRAP_ARM_TUNING_ENABLED);
    }

    @Override
    public void periodic() {
        mechanism.update();
        extensionPosition = linearServo.getPosition() * ARM_SERVO_MAX_MM;

        linearServo.setPosition(Math.max(0, extensionTarget / ARM_SERVO_MAX_MM));
        SmartDashboard.putNumber("Arm Position", extensionPosition);
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
        extensionTarget = mm;
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