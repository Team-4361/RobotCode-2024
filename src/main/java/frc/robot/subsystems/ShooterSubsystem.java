package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;
import frc.robot.util.motor.MotorModel;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.SHOOTER_MOTOR_1_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_2_ID;

public class ShooterSubsystem extends SubsystemBase {
    // two variables: one for each motor. Remember the type is "FRCSparkMax"
    // remember to make it final and initialize it in the constructor
    private final FRCSparkMax motor1;
    private final FRCSparkMax motor2;

    public double getVelocity() {
        return (motor1.getEncoder().getVelocity() + motor2.getEncoder().getVelocity()) / 2;
    }

    public boolean isNoPower() { return motor1.get() == 0 && motor2.get() == 0; }

    public ShooterSubsystem() {
        // initialize your two motor variables here with values in the Constants file
        // you need a Constant for each ID. Assume both are "kBrushless" without a constant.
        // motor1 = *****;
        motor1 = new FRCSparkMax(SHOOTER_MOTOR_1_ID, kBrushless, MotorModel.NEO);
        motor2 = new FRCSparkMax(SHOOTER_MOTOR_2_ID, kBrushless, MotorModel.NEO);
    }

    // Add your methods here to move and stop the motor. I would recommend making
    // one with an input of the motor speed which will be filled in from a Command
    // using the made constants values.
    public void moveMotor(double speed){
        motor1.set(speed);
        motor2.set(speed);
    }

    public void stopMotor(){
        motor1.set(0);
        motor2.set(0);
    }

    @Override
    public void periodic() {
        // You can use SmartDashboard to update some fields if you would like
        SmartDashboard.putString("Motor 1 Speed: ", ""+motor1.get());
        SmartDashboard.putString("Motor 2 Speed: ", ""+motor2.get());
    }
}
