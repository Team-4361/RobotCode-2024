package frc.robot.subsystems;

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
public class WristSubsystem extends PIDRotationalMechanism implements LoggableInputs, Subsystem {
    private final Servo linearServo;

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
                RotationUnit.DEGREES
        );
        this.linearServo = new Servo(WRIST_SERVO_ID);
        linearServo.setBoundsMicroseconds(
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
        Constants.runIfNotReplay(() -> extensionPosition = linearServo.getPosition() * WRIST_SERVO_MAX_MM);
        super.update();

        linearServo.setPosition(Math.max(0, extensionTarget / WRIST_SERVO_MAX_MM));
    }

    /**
     * Updates a LogTable with the data to log.
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        super.toLog(table);
        table.put("ExtensionMM", extensionPosition);
        table.put("TargetMM", extensionTarget);
    }

    /**
     * Sets the extension target of the {@link WristSubsystem}.
     * @param mm The extension target in <b>millimeters</b>.
     */
    public void setExtensionTarget(double mm) {
        Constants.runIfNotReplay(() -> {
            this.extensionTarget = mm;
        });
    }

    /** Extends the {@link WristSubsystem} to the maximum allowed value. */
    public void extendWrist() { setExtensionTarget(WRIST_SERVO_MAX_MM); }

    /** Retracts the {@link WristSubsystem} to the minimum allowed value. */
    public void retractWrist() { setExtensionTarget(0); }

    /**
     * Updates data based on a LogTable.
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        super.fromLog(table);
        this.extensionPosition = table.get("ExtensionMM", this.extensionPosition);
        this.extensionTarget = table.get("TargetMM", this.extensionTarget);
    }
}
