package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;
import frc.robot.util.preset.PresetMap;

import static frc.robot.Constants.Debug.WRIST_TUNING_ENABLED;
import static frc.robot.Constants.Wrist.*;

/**
 * This {@link TrapWristSubsystem} is designed to control the {@link Robot}'s wrist. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for turning.
 */
public class TrapWristSubsystem extends SubsystemBase {
    private final Servo grabServo;
    private final PIDRotationalMechanism mechanism;

    public double extensionPosition = 0.0;
    public double extensionTarget = 0.0;

    /** Constructs a new {@link PIDRotationalMechanism}. */
    public TrapWristSubsystem() {
        mechanism = new PIDRotationalMechanism(
                WRIST_MOTOR_ID,
                WRIST_PID,
                WRIST_KS,
                WRIST_KV,
                WRIST_KA,
                MotorModel.NEO_550,
                "Wrist",
                WRIST_TUNING_ENABLED,
                WRIST_TURN_RATIO,
                RotationUnit.DEGREES,
                false
        );

        grabServo = new Servo(WRIST_SERVO_ID);
        grabServo.setBoundsMicroseconds(
                WRIST_MAX_US,
                WRIST_DEAD_BAND_MAX_US,
                WRIST_CENTER_US,
                WRIST_DEAD_BAND_MIN_US,
                WRIST_MIN_US
        );

        mechanism.setInverted(WRIST_INVERTED);
    }

    public void registerPresets(PresetMap<Double> map) { mechanism.registerPresets(map); }

    @Override
    public void periodic() {
        extensionPosition = grabServo.getPosition() * WRIST_SERVO_MAX_MM;
        mechanism.update();
        grabServo.setPosition(Math.max(0, extensionTarget / WRIST_SERVO_MAX_MM));
        if (WRIST_TUNING_ENABLED)
            SmartDashboard.putNumber("Wrist: EXT Pos", getExtensionPosition());
    }

    /**
     * Sets the extension target of the {@link TrapWristSubsystem}.
     * @param mm The extension target in <b>millimeters</b>.
     */
    public void setExtensionTarget(double mm) { this.extensionTarget = mm; }

    public void translateWrist(double speed) {
        mechanism.translateMotor(speed);
    }

    /** Extends the {@link TrapWristSubsystem} to the maximum allowed value -- dropping the note. */
    public void dropNote() { setExtensionTarget(WRIST_SERVO_MAX_MM); }

    /** Retracts the {@link TrapWristSubsystem} to the minimum allowed value -- grabbing the note. */
    public void grabNote() { setExtensionTarget(0); }

    public double getExtensionPosition(){ return extensionPosition; }
}
