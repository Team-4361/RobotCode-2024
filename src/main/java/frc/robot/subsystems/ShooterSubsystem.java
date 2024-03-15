package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.math.GearRatio;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.IUpdatable;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;

import static frc.robot.Constants.Debug.SHOOTER_TUNING_ENABLED;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.util.math.GlobalUtils.inTolerance;

/**
 * This {@link ShooterSubsystem} is designed to enable the {@link Robot} to shoot. Physically, this
 * mechanism contains two motors which need to be driven opposite to each other.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final PIDRotationalMechanism leftShooter;
    private final PIDRotationalMechanism rightShooter;
    private final DashTunableNumber shootTune;
    private final DashTunableNumber delayTune;

    private double defaultRPM = SHOOT_RPM;
    private long delayMs = SHOOT_END_DELAY_MS;
    private double targetRPM = 0;

    /**Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values. */
    public ShooterSubsystem() {

        if (SHOOTER_TUNING_ENABLED) {
            shootTune = new DashTunableNumber("Shooter: Speed", SHOOT_RPM);
            delayTune = new DashTunableNumber("Shooter: Delay", SHOOT_END_DELAY_MS);
            shootTune.addConsumer(this::setDefaultRPM);
            delayTune.addConsumer(this::setDelayMS);
        } else {
            shootTune = null;
            delayTune = null;
        }

        this.leftShooter = new PIDRotationalMechanism(
                SHOOT_LEFT_MOTOR_ID,
                SHOOT_PID,
                MotorModel.NEO,
                "LeftShooter",
                SHOOTER_TUNING_ENABLED,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );

        this.rightShooter = new PIDRotationalMechanism(
                SHOOT_RIGHT_MOTOR_ID,
                SHOOT_PID,
                MotorModel.NEO,
                "RightShooter",
                SHOOTER_TUNING_ENABLED,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );

        leftShooter.setInverted(true);
        rightShooter.setInverted(false);
    }

    public double getShooterRPM() {
        return GlobalUtils.averageDouble(Math.abs(leftShooter.getVelocity()), Math.abs(rightShooter.getVelocity()));
    }

    public long getDelayMS() { return this.delayMs; }

    public void setDelayMS(double delayMs) { this.delayMs = (long)delayMs; }
    public void setDefaultRPM(double rpm) { this.defaultRPM = rpm; }

    public void translateMotor(double speed) {
        leftShooter.translateMotor(speed);
        rightShooter.translateMotor(speed);
    }

    public void startTargetRPM(double rpm) {
        this.targetRPM = rpm;
        leftShooter.setTarget(rpm);
        rightShooter.setTarget(rpm);
    }

    public boolean atTarget(double tolerance) { return inTolerance(targetRPM, getShooterRPM(), tolerance); }
    public void startTargetToDefault() { startTargetRPM(defaultRPM); }

    public void stop() {
        if (!DriverStation.isAutonomous())
            startTargetRPM(0);
    }

    @Override
    public void periodic() {
        IUpdatable.updateAll(shootTune, delayTune, leftShooter, rightShooter);
    }
}