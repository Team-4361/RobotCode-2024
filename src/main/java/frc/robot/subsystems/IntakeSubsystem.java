package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Debug.INTAKE_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.*;
import static frc.robot.Constants.Indexer.INDEX_RIGHT_MOTOR_ID;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final FRCSparkMax motor;
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
        this.motor = new FRCSparkMax(INTAKE_MOTOR_ID, kBrushless, MotorModel.NEO_550);
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

    public void start() {
        motor.set(targetSpeed);
    }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() {
        motor.stopMotor();
    }

    public boolean hasNote() { return !sensorActivated; }
}




