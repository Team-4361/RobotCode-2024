package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Map;

import static frc.robot.Constants.Debug.INTAKE_TUNING_ENABLED;
import static frc.robot.Constants.Intake.*;
import static java.util.Map.entry;

public class IntakeSubsystem extends BaseSubsystem {
    private final DigitalInput sensor;
    private boolean sensorActivated;

    public IntakeSubsystem(){
        super(
                "Intake",
                INTAKE_SPEED,
                INTAKE_TUNING_ENABLED,
                Map.ofEntries(
                        entry(INTAKE_MOTOR_ID, false)
                )
        );
        this.sensor = new DigitalInput(INTAKE_SENSOR_PORT);
        setDashUpdate(()-> {
            if (!RobotBase.isSimulation()) {
                sensorActivated = sensor.get();
            }

            SmartDashboard.putBoolean("Intake: Has Note", hasNote());
        });
    }
    public boolean hasNote() { return !sensorActivated; }
}