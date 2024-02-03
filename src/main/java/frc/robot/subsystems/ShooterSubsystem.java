package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.pid.PIDWheelModule;
import org.littletonrobotics.junction.Logger;

import static frc.robot.Constants.Control.SHOOTER_TUNING_ENABLED;
import static frc.robot.Constants.Shooter.*;

/**
 * This {@link ShooterSubsystem} is designed to enable the {@link Robot} to shoot. Physically, this
 * mechanism contains two motors which need to be driven opposite to each other.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final PIDWheelModule leftWheel;
    private final PIDWheelModule rightWheel;

    /**Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values. */
    public ShooterSubsystem() {
        String tuneName = SHOOTER_TUNING_ENABLED ? "Shooter: PID" : "";
        this.leftWheel = new PIDWheelModule(
                LEFT_SHOOTER_MOTOR_ID,
                SHOOT_PID,
                SHOOT_KS,
                SHOOT_KV,
                SHOOT_KA,
                "LeftShooter",
                tuneName
        );
        this.rightWheel = new PIDWheelModule(
                RIGHT_SHOOTER_MOTOR_ID,
                SHOOT_PID,
                SHOOT_KS,
                SHOOT_KV,
                SHOOT_KA,
                "RightShooter",
                tuneName
        );
    }

    /** @return If the {@link ShooterSubsystem} is at target. */
    public boolean atTarget() { return leftWheel.atTarget() && rightWheel.atTarget(); }

    @Override
    public void periodic() {
        leftWheel.update();
        rightWheel.update();
    }

    /**
     * Sets the target of the {@link ShooterSubsystem}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) {
        leftWheel.setTarget(rpm);
        rightWheel.setTarget(-rpm);
    }

    /** Stops the {@link ShooterSubsystem} from spinning. */
    public void stop() { setTarget(0); }
}