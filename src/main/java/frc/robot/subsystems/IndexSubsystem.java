package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.*;

public class IndexSubsystem extends SubsystemBase {
    private final FRCSparkMax motor1, motor2;
    private final ColorSensorV3 sensor;

    public IndexSubsystem(){
        motor1 = new FRCSparkMax(INDEX_MOTOR_1_ID, kBrushless, MotorModel.NEO);
        motor2 = new FRCSparkMax(INDEX_MOTOR_2_ID, kBrushless, MotorModel.NEO);
        sensor = new ColorSensorV3(INDEX_SENSOR_PORT);
    }

    //methods using motor.set()
    public void moveIndexer(double speed){
        motor1.set(speed);
        motor2.set(speed);
    }
    public void stopIndexer(){
        motor1.set(0);
        motor2.set(0);
    }
    public boolean hasNote(){
        return sensor.getRed() > RED_MINIMUM_TOLERANCE && sensor.getRed() < RED_MAXIMUM_TOLERANCE
                && sensor.getBlue() > BLUE_MINIMUM_TOLERANCE && sensor.getBlue() < BLUE_MAXIMUM_TOLERANCE
                && sensor.getGreen() > GREEN_MINIMUM_TOLERANCE && sensor.getGreen() < GREEN_MAXIMUM_TOLERANCE;
    }
    //SmartDashboard
    @Override
    public void periodic(){
        SmartDashboard.putString("Indexer 1 Speed: ", ""+motor1.get());
        SmartDashboard.putString("Indexer 2 Speed: ", ""+motor2.get());

    }
}
