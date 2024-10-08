package frc.robot.subsystems;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.base.BaseSubsystem;

import java.util.Map;

import static edu.wpi.first.math.filter.Debouncer.DebounceType.kBoth;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Systems.INTAKE;
import static java.util.Map.entry;

public class IntakeSubsystem extends BaseSubsystem {
    private final DigitalInput sensor;
    private final Debouncer debouncer;
    private boolean sensorActivated = false;

    public IntakeSubsystem(){
        super(INTAKE, Map.ofEntries(
                entry(INTAKE_MOTOR_ID, false))
        );
        this.debouncer = new Debouncer(0.05, kBoth);

        if (isEnabled()) {
            this.sensor = new DigitalInput(INTAKE_SENSOR_PORT);

            setDashUpdate(() -> {
                if (!RobotBase.isSimulation()) {
                    sensorActivated = debouncer.calculate(sensor.get());
                }

                SmartDashboard.putBoolean("Intake: Has Note", hasNote());
            });
        } else {
            this.sensor = null;
        }
        registerConstant("Speed", INTAKE_SPEED);
    }
    public boolean hasNote() { return !sensorActivated; }
    public void startNormal() { startAll(getTargetSpeed()); }
    public void startReverse() { startAll(-getTargetSpeed());}
    
    public void setTargetSpeed(double speed) { setConstant("Speed", speed); }
    public double getTargetSpeed() { return getConstant("Speed"); }
}