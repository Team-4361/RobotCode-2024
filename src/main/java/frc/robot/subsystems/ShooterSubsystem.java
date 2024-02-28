package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.pid.DashTunableNumber;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Debug.SHOOTER_TUNING_ENABLED;
import static frc.robot.Constants.Shooter.*;

/**
 * This {@link ShooterSubsystem} is designed to enable the {@link Robot} to shoot. Physically, this
 * mechanism contains two motors which need to be driven opposite to each other.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final DashTunableNumber shootTune;
    private final DashTunableNumber delayTune;
    private double targetSpeed = SHOOT_SPEED;
    private long delayMs = SHOOT_END_DELAY_MS;

    /**Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values. */
    public ShooterSubsystem() {
        if (SHOOTER_TUNING_ENABLED) {
            shootTune = new DashTunableNumber("Shooter: Speed", SHOOT_SPEED);
            delayTune = new DashTunableNumber("Shooter: Delay", SHOOT_END_DELAY_MS);
            shootTune.addConsumer(this::setTargetSpeed);
            delayTune.addConsumer(this::setDelayMS);
        } else {
            shootTune = null;
            delayTune = null;
        }

        this.leftMotor = new CANSparkMax(SHOOT_LEFT_MOTOR_ID, kBrushless);
        this.rightMotor = new CANSparkMax(SHOOT_RIGHT_MOTOR_ID, kBrushless);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    }

    public long getDelayMS() { return this.delayMs; }

    public void setDelayMS(double delayMs) { this.delayMs = (long)delayMs; }
    public void setTargetSpeed(double speed) { this.targetSpeed = speed; }

    @Override
    public void periodic() {
        if (shootTune != null)
            shootTune.update();
        if (delayTune != null)
            delayTune.update();
    }

    /**
     * Sets the target of the {@link ShooterSubsystem} to the Shoot RPM.
     */
    public void start() {
        leftMotor.set(targetSpeed);
        rightMotor.set(targetSpeed);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}