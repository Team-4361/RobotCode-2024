package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.TimedDigitalInput;
import frc.robot.util.pid.TunableNumber;

import java.util.Map;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;
import static java.util.Map.entry;

public class ClimberSubsystem extends BaseSubsystem {
    private final TimedDigitalInput leftSensor;
    private final TimedDigitalInput rightSensor;


    public ClimberSubsystem(){
        super("Climber",
                CLIMBER_SPEED,
                CLIMBER_TUNING_ENABLED,
                Map.ofEntries(
                        entry(CLIMBER_LEFT_ID, CLIMBER_LEFT_INVERTED),
                        entry(CLIMBER_RIGHT_ID,CLIMBER_RIGHT_INVERTED))
        );
        this.leftSensor = new TimedDigitalInput(CLIMBER_LEFT_DIO);
        this.rightSensor = new TimedDigitalInput(CLIMBER_RIGHT_DIO);
    }

    public long getLeftActivatedDuration()  { return leftSensor.getActivatedDuration();  }
    public long getRightActivatedDuration() { return rightSensor.getActivatedDuration(); }


    public boolean isLeftRetracted() { return leftSensor.get(); }
    public boolean isRightRetracted() { return rightSensor.get(); }


    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        leftSensor.update();
        rightSensor.update();

        SmartDashboard.putBoolean("Climber: Left Down", leftSensor.get());
        SmartDashboard.putBoolean("Climber: Right Down", rightSensor.get());
    }
}
