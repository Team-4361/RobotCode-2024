package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.base.BaseSubsystem;

import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.Systems.SHOOTER;
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
        super(SHOOTER, ofEntries(
                entry(SHOOT_LEFT_MOTOR_ID, true),
                entry(SHOOT_RIGHT_MOTOR_ID, false))
        );
        registerConstant("DelayMS", SHOOT_END_DELAY_MS);
        registerConstant("Speed", SHOOT_SPEED);
    }

    public long getDelayMS() { return (long) getConstant("DelayMS"); }
    public double getTargetSpeed() { return getConstant("Speed"); }
    public boolean atTarget(double rpm) { return getRPM() >= rpm;}
    public void start() { this.startAll(getTargetSpeed()); }
    public void stop() { this.stopAll(); }

    public void setTargetSpeed(double speed) { setConstant("Speed", speed); }

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

    /** Sets the target of the {@link ShooterSubsystem} to the Shoot RPM. */
    public void setEnabled(boolean enabled) { this.fireMode = enabled; }
}