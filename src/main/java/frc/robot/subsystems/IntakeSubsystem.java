package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import static frc.robot.Constants.Debug.INTAKE_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.INDEX_SENSOR_PORT;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase implements LoggableInputs {
    private final PIDMechanismBase intakeWheel;
    private final DigitalInput sensor;
    private final ColorSensorV3 colorSensor;
    private boolean sensorActivated = false;

    private final DashTunableNumber rTune;
    private final DashTunableNumber gTune;
    private final DashTunableNumber bTune;
    private final DashTunableNumber intakeTune;
    private double targetRPM = INTAKE_RPM;
    private boolean stopped = true;

    private double rValue, gValue, bValue;

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
        rTune = new DashTunableNumber("Intake: R Min", 100);
        gTune = new DashTunableNumber("Intake: G Tune", 100);
        bTune = new DashTunableNumber("Intake: B Tune", 100);
        sensor = new DigitalInput(INDEX_SENSOR_PORT);
        colorSensor = new ColorSensorV3(I2C.Port.kOnboard);
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

        if (!RobotBase.isSimulation() && !Constants.isReplay())
            sensorActivated = sensor.get();

        rValue = colorSensor.getRed();
        gValue = colorSensor.getGreen();
        bValue = colorSensor.getBlue();

        Logger.processInputs("Index", this);
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

    public boolean hasNote() { return sensorActivated; }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("SensorActivated", sensorActivated);
        table.put("Red", rValue);
        table.put("Green", gValue);
        table.put("Blue", bValue);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.sensorActivated = table.get("SensorActivated", sensorActivated);
        this.rValue = table.get("Red", rValue);
        this.gValue = table.get("Green", gValue);
        this.bValue = table.get("Blue", bValue);
    }
}




