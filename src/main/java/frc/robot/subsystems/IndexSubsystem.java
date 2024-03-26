package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.pid.TunableNumber;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.*;

public class IndexSubsystem extends SubsystemBase {
    private final TunableNumber indexTune;
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private double targetSpeed = INDEX_SPEED;

    public IndexSubsystem() {
        if (INDEX_TUNING_ENABLED) {
            indexTune = new TunableNumber("Index: Speed", INDEX_SPEED);
            indexTune.addConsumer(this::setTargetSpeed);
        } else {
            indexTune = null;
        }
        this.leftMotor = new CANSparkMax(INDEX_LEFT_MOTOR_ID, kBrushless);
        this.rightMotor = new CANSparkMax(INDEX_RIGHT_MOTOR_ID, kBrushless);

        leftMotor.setInverted(true);
        rightMotor.setInverted(false);
    }

    public void setTargetSpeed(double speed) { this.targetSpeed = speed; }

    @Override
    public void periodic() {
        if (indexTune != null)
            indexTune.update();

        SmartDashboard.putNumber("Index: Left Amps", leftMotor.getOutputCurrent());
        SmartDashboard.putNumber("Index: Right Amps", rightMotor.getOutputCurrent());
    }

    public void startReverse() {
        leftMotor.set(-targetSpeed);
        rightMotor.set(-targetSpeed);
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     */
    public void startNormal() {
        leftMotor.set(targetSpeed);
        rightMotor.set(targetSpeed);
    }

    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }
}


