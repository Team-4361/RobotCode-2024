package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.pid.DashTunableNumber;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Debug.INTAKE_TUNING_ENABLED;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final CANSparkMax motor;
    private final DashTunableNumber intakeTune;
    private final DigitalInput sensor;
    private double targetSpeed = INTAKE_SPEED;
    private boolean sensorActivated;

    public IntakeSubsystem() {
        if (INTAKE_TUNING_ENABLED) {
            intakeTune = new DashTunableNumber("Intake: Speed", INTAKE_SPEED);
            intakeTune.addConsumer(this::setTargetSpeed);
        } else {
            intakeTune = null;
        }
        this.sensor = new DigitalInput(INTAKE_SENSOR_PORT);
        this.motor = new CANSparkMax(INTAKE_MOTOR_ID, kBrushless);
    }

    public void setTargetSpeed(double speed) { this.targetSpeed = speed; }

    @Override
    public void periodic() {
        if (intakeTune != null)
            intakeTune.update();

        if (!RobotBase.isSimulation()) {
            sensorActivated = sensor.get();
        }

        SmartDashboard.putBoolean("Intake: Has Note", hasNote());
    }

    public void startNormal() { motor.set(targetSpeed); }
    public void startReverse() { motor.set(-targetSpeed); }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() {
        motor.stopMotor();
    }

    public boolean hasNote() { return !sensorActivated; }
}