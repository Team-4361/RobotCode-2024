package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;

public class ClimberSubsystem extends SubsystemBase {
    private final FRCSparkMax leftMotor;
    private final FRCSparkMax rightMotor;
    private final DigitalInput leftSensor;
    private final DigitalInput rightSensor;

    public ClimberSubsystem(){
        this.leftMotor = new FRCSparkMax(CLIMBER_LEFT_ID, kBrushless, MotorModel.NEO_550);
        this.rightMotor = new FRCSparkMax(CLIMBER_RIGHT_ID, kBrushless, MotorModel.NEO_550);
        this.leftSensor = new DigitalInput(CLIMBER_LEFT_DIO);
        this.rightSensor = new DigitalInput(CLIMBER_RIGHT_DIO);
    }

    public void

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        if (RobotBase.isSimulation()) {
            leftMotor.updateSim();
            rightMotor.updateSim();
        }
    }
}
