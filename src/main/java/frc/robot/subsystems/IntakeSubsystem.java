package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Intake.INTAKE_MOTOR_ID;

public class IntakeSubsystem extends SubsystemBase {
    private final FRCSparkMax motor;

    public IntakeSubsystem() {
       motor = new FRCSparkMax(INTAKE_MOTOR_ID, kBrushless, MotorModel.NEO);
    }

    //methods
    public void moveIntake(double speed)
    {
        motor.set(speed);
    }
    public void stopIntake()
    {
        motor.set(0);
    }

    @Override
    public void periodic(){
        SmartDashboard.putString("Intake Speed:", ""+motor.get());
    }

}
