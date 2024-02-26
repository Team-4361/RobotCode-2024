package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;

public class ClimberSubsystem extends SubsystemBase {
    private final FRCSparkMax leftMotor;
    private final FRCSparkMax rightMotor;
    private final DigitalInput leftSensor;
    private final DigitalInput rightSensor;
    private final DashTunableNumber speedTune;
    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;
    private double targetSpeed = CLIMBER_SPEED;

    public void setTargetSpeed(double speed) { this.targetSpeed = speed; }

    public ClimberSubsystem(){
        this.leftMotor = new FRCSparkMax(CLIMBER_LEFT_ID, kBrushless, MotorModel.NEO_550);
        this.rightMotor = new FRCSparkMax(CLIMBER_RIGHT_ID, kBrushless, MotorModel.NEO_550);
        this.leftSensor = new DigitalInput(CLIMBER_LEFT_DIO);
        this.rightSensor = new DigitalInput(CLIMBER_RIGHT_DIO);

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

    public void reset() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }

    public boolean isLeftRetracted() { return leftSensor.get(); }
    public boolean isRightRetracted() { return rightSensor.get(); }

    public void moveLeftUp() { leftMotor.set(targetSpeed); }
    public void moveRightUp() { rightMotor.set(targetSpeed); }
    public void moveLeftDown() { leftMotor.set(-targetSpeed); }
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
        if (RobotBase.isSimulation()) {
            leftMotor.updateSim();
            rightMotor.updateSim();
        }
        if (speedTune != null)
            speedTune.update();

        SmartDashboard.putNumber("Climber: Left Position", leftEncoder.getPosition());
        SmartDashboard.putNumber("Climber: Right Position", rightEncoder.getPosition());
    }
}