package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;

import static frc.robot.Constants.Debug.INTAKE_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.INDEX_SENSOR_PORT;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final PIDMechanismBase intakeWheel;
    private final DashTunableNumber intakeTune;
    private final DigitalInput sensor;

    private boolean sensorActivated = false;
    private double targetRPM = INTAKE_RPM;
    private boolean stopped = true;

    public IntakeSubsystem() {
        String tuneName = INTAKE_TUNING_ENABLED ? "Intake: PID" : "";
        intakeWheel = new PIDRotationalMechanism(
                INTAKE_MOTOR_ID,
                INTAKE_PID,
                INTAKE_KS,
                INTAKE_KV,
                INTAKE_KA,
                MotorModel.NEO,
                "Intake",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );
        sensor = new DigitalInput(INDEX_SENSOR_PORT);
        if (INTAKE_TUNING_ENABLED) {
            intakeTune = new DashTunableNumber("Intake: Speed", INTAKE_RPM);
            intakeTune.addConsumer(this::setTargetRPM);
        } else {
            intakeTune = null;
        }
    }

    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }

    /** @return If the {@link IndexSubsystem} is at target. */
    public boolean atTarget() { return intakeWheel.atTarget(); }

    @Override
    public void periodic() {
        intakeWheel.update();
        if (intakeTune != null && !stopped)
            intakeTune.update();

        if (!RobotBase.isSimulation())
            sensorActivated = sensor.get();

        SmartDashboard.putBoolean("Has Note", hasNote());
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     */
    public void setTarget(double rpm) { intakeWheel.setTarget(INTAKE_INVERTED ? -rpm: rpm); }

    public void start() {
        intakeWheel.setTarget(targetRPM);
        stopped = targetRPM == 0;
    }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() { intakeWheel.stop(); stopped = true; }

    public boolean hasNote() { return !sensorActivated; }
}




