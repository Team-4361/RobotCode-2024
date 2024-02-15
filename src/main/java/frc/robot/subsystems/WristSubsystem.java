package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDRotationalMechanism;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static frc.robot.Constants.Debug.WRIST_TUNING_ENABLED;
import static frc.robot.Constants.Wrist.*;

/**
 * This {@link WristSubsystem} is designed to control the {@link Robot}'s wrist. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for turning.
 */
public class WristSubsystem extends PIDRotationalMechanism implements Subsystem {
    private final Servo grabServo;

    public double extensionPosition = 0.0;
    public double extensionTarget = 0.0;

    /** Constructs a new {@link PIDRotationalMechanism}. */
    public WristSubsystem() {
        super(
                WRIST_MOTOR_ID,
                WRIST_PID,
                WRIST_KS,
                WRIST_KV,
                WRIST_KA,
                MotorModel.NEO_550,
                "Wrist",
                WRIST_TUNING_ENABLED ? "Wrist: PID" : "",
                WRIST_TURN_RATIO,
                RotationUnit.DEGREES,
                false
        );
        this.grabServo = new Servo(WRIST_SERVO_ID);
        grabServo.setBoundsMicroseconds(
                WRIST_MAX_US,
                WRIST_DEAD_BAND_MAX_US,
                WRIST_CENTER_US,
                WRIST_DEAD_BAND_MIN_US,
                WRIST_MIN_US
        );

        CommandScheduler.getInstance().registerSubsystem(this);
    }

    @Override
    public void periodic() {
        extensionPosition = grabServo.getPosition() * WRIST_SERVO_MAX_MM;
        super.update();
        grabServo.setPosition(Math.max(0, extensionTarget / WRIST_SERVO_MAX_MM));
    }

    /**
     * Sets the extension target of the {@link WristSubsystem}.
     * @param mm The extension target in <b>millimeters</b>.
     */
    public void setExtensionTarget(double mm) {
        this.extensionTarget = mm;
    }

    /** Extends the {@link WristSubsystem} to the maximum allowed value. */
    public void extendWrist() { setExtensionTarget(WRIST_SERVO_MAX_MM); }

    /** Retracts the {@link WristSubsystem} to the minimum allowed value. */
    public void retractWrist() { setExtensionTarget(0); }
}
