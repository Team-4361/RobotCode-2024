package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.*;

public class IndexSubsystem extends SubsystemBase {
    private final DashTunableNumber indexTune;
    private final FRCSparkMax leftMotor;
    private final FRCSparkMax rightMotor;
    private double targetSpeed = INDEX_SPEED;

    public IndexSubsystem() {
        if (INDEX_TUNING_ENABLED) {
            indexTune = new DashTunableNumber("Index: Speed", INDEX_SPEED);
            indexTune.addConsumer(this::setTargetSpeed);
        } else {
            indexTune = null;
        }
        this.leftMotor = new FRCSparkMax(INDEX_LEFT_MOTOR_ID, kBrushless, MotorModel.NEO_550);
        this.rightMotor = new FRCSparkMax(INDEX_RIGHT_MOTOR_ID, kBrushless, MotorModel.NEO_550);
    }

    public void setTargetSpeed(double speed) { this.targetSpeed = speed; }

    @Override
    public void periodic() {
        if (indexTune != null)
            indexTune.update();
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
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



