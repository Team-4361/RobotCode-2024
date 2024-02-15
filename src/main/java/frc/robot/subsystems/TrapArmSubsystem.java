package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDLinearMechanism;
import frc.robot.util.pid.PIDRotationalMechanism;

import static frc.robot.Constants.Arm.*;
import static frc.robot.Constants.Debug.TRAP_ARM_TUNING_ENABLED;

/**
 * This {@link TrapArmSubsystem} is designed to control the {@link Robot}'s arm. It has an Actuonix L16-50-35-6R
 * Linear Servo for grabbing, and a 63:1 NEO-550 motor used for extending.
 */
public class TrapArmSubsystem extends PIDLinearMechanism implements Subsystem {
    private final Servo linearServo;
    public double extensionPosition = 0.0;
    public double extensionTarget = 0.0;

    /** Constructs a new {@link PIDRotationalMechanism}. */
    public TrapArmSubsystem() {
        super(
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
        CommandScheduler.getInstance().registerSubsystem(this);
        setTuneMode(TRAP_ARM_TUNING_ENABLED);
    }

    @Override
    public void periodic() {
        extensionPosition = linearServo.getPosition() * ARM_SERVO_MAX_MM;
        super.update();

        linearServo.setPosition(Math.max(0, extensionTarget / ARM_SERVO_MAX_MM));
        SmartDashboard.putNumber("Arm Position", extensionPosition);
    }

    public void translateAngle(double speed) {
        linearServo.set(speed);
    }
}