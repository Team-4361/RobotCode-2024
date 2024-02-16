package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;

import static frc.robot.Constants.Debug.SHOOTER_TUNING_ENABLED;
import static frc.robot.Constants.Shooter.*;

/**
 * This {@link ShooterSubsystem} is designed to enable the {@link Robot} to shoot. Physically, this
 * mechanism contains two motors which need to be driven opposite to each other.
 */
public class ShooterSubsystem extends SubsystemBase {
    private final PIDMechanismBase leftWheel;
    private final PIDMechanismBase rightWheel;
    private final DashTunableNumber shootTune;
    private double targetRPM = SHOOT_RPM;
    private boolean stopped = true;

    /**Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values. */
    public ShooterSubsystem() {
        String tuneName = SHOOTER_TUNING_ENABLED ? "Shooter: PID" : "";
        this.leftWheel = new PIDRotationalMechanism(
                LEFT_SHOOTER_MOTOR_ID,
                SHOOT_PID,
                SHOOT_KS,
                SHOOT_KV,
                SHOOT_KA,
                MotorModel.NEO,
                "LeftShooter",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );
        this.rightWheel = new PIDRotationalMechanism(
                RIGHT_SHOOTER_MOTOR_ID,
                SHOOT_PID,
                SHOOT_KS,
                SHOOT_KV,
                SHOOT_KA,
                MotorModel.NEO,
                "RightShooter",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );

        leftWheel.setTolerance(SHOOT_RPM_TOLERANCE);
        rightWheel.setTolerance(SHOOT_RPM_TOLERANCE);

        if (SHOOTER_TUNING_ENABLED) {
            shootTune = new DashTunableNumber("Shooter: Speed", SHOOT_RPM);
            shootTune.addConsumer(this::setTargetRPM);
        } else {
            shootTune = null;
        }
    }

    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }

    /** @return If the {@link ShooterSubsystem} is at target. */
    public boolean atTarget() { return leftWheel.atTarget() && rightWheel.atTarget(); }

    @Override
    public void periodic() {
        leftWheel.update();
        rightWheel.update();
        if (shootTune != null && !stopped)
            shootTune.update();
    }

    //private final Spark LED_1 = new Spark(0);


    /**
     * Sets the target of the {@link ShooterSubsystem} to the Shoot RPM.
     */
    public void start() {
        leftWheel.setTarget(targetRPM);
        rightWheel.setTarget(targetRPM);
        stopped = targetRPM == 0;
    }

    /** Stops the {@link ShooterSubsystem} from spinning. */
    public void stop() { leftWheel.stop(); rightWheel.stop(); stopped = true; }
}