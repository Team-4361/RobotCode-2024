package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

import static frc.robot.Constants.Debug.SHOOTER_TUNING_ENABLED;
import static frc.robot.Constants.Shooter.*;
import static java.util.Map.entry;
import static java.util.Map.ofEntries;

/**
 * This {@link ShooterSubsystem} is designed to enable the {@link Robot} to shoot. Physically, this
 * mechanism contains two motors which need to be driven opposite to each other.
 */
public class ShooterSubsystem extends BaseSubsystem {
    private boolean fireMode = false;

    /**Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values. */
    public ShooterSubsystem() {
        super(
                "Shooter",
                SHOOT_SPEED,
                SHOOTER_TUNING_ENABLED,
                ofEntries(
                        entry(SHOOT_LEFT_MOTOR_ID, true),
                        entry(SHOOT_RIGHT_MOTOR_ID, false)
                )
        );
        registerConstant("DelayMS", SHOOT_END_DELAY_MS);
    }

    public long getDelayMS() { return (long) getConstant("DelayMS"); }
    public boolean atTarget(double rpm) { return getRPM() >= rpm;}

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shooter RPM", getRPM());
        SmartDashboard.putBoolean("Shooter At Target", atTarget(5000));

        if (DriverStation.isAutonomousEnabled()) {
            start();
        } else {
            if (fireMode) {
                start();
            } else {
                stop();
            }
        }
    }

    /**
     * Sets the target of the {@link ShooterSubsystem} to the Shoot RPM.
     */
    public void setEnabled(boolean enabled) {
        this.fireMode = enabled;
    }
}