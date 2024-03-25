package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.TimedDigitalInput;
import frc.robot.util.pid.DashTunableNumber;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;

public class ClimberSubsystem extends SubsystemBase {
    private final CANSparkMax leftMotor;
    private final CANSparkMax rightMotor;
    private final TimedDigitalInput leftSensor;
    private final TimedDigitalInput rightSensor;
    private final DashTunableNumber speedTune;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private double targetSpeed = CLIMBER_SPEED;

    public void setTargetSpeed(double speed) { this.targetSpeed = speed; }

    public ClimberSubsystem(){
        this.leftMotor = new CANSparkMax(CLIMBER_LEFT_ID, kBrushless);
        this.rightMotor = new CANSparkMax(CLIMBER_RIGHT_ID, kBrushless);
        this.leftSensor = new TimedDigitalInput(CLIMBER_LEFT_DIO);
        this.rightSensor = new TimedDigitalInput(CLIMBER_RIGHT_DIO);

        leftMotor.setIdleMode(kBrake);
        rightMotor.setIdleMode(kBrake);

        this.leftEncoder = leftMotor.getEncoder();
        this.rightEncoder = rightMotor.getEncoder();

        leftMotor.setInverted(CLIMBER_LEFT_INVERTED);
        rightMotor.setInverted(CLIMBER_RIGHT_INVERTED);

        if (CLIMBER_TUNING_ENABLED) {
            speedTune = new DashTunableNumber("Climber: Speed", CLIMBER_SPEED);
            speedTune.addConsumer(this::setTargetSpeed);
        } else {
            speedTune = null;
        }
    }

    public long getLeftActivatedDuration()  { return leftSensor.getActivatedDuration();  }
    public long getRightActivatedDuration() { return rightSensor.getActivatedDuration(); }

    public void reset() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public boolean isLeftRetracted() { return leftSensor.get(); }
    public boolean isRightRetracted() { return rightSensor.get(); }

    public void moveLeftUp()    { leftMotor.set(targetSpeed);   }
    public void moveRightUp()   { rightMotor.set(targetSpeed);  }
    public void moveLeftDown()  { leftMotor.set(-targetSpeed);  }
    public void moveRightDown() { rightMotor.set(-targetSpeed); }

    public void stopLeft() { leftMotor.stopMotor(); }
    public void stopRight() { rightMotor.stopMotor(); }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        if (speedTune != null)
            speedTune.update();
        leftSensor.update();
        rightSensor.update();

        SmartDashboard.putBoolean("Climber: Left Down", leftSensor.get());
        SmartDashboard.putBoolean("Climber: Right Down", rightSensor.get());
    }
}
