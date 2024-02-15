package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
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
import org.littletonrobotics.junction.inputs.LoggableInputs;


import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.*;

//** This {@link IndexSubsystem} is designed to hold the note between the intake and the shooter to allow shooter to
// fully speed up for shooting note at desired speed. Uses 2 NEO 550s spinning slowly opposite of each other. Hands off
// note to shooter when shooter is at desired speed.

public class IndexSubsystem extends SubsystemBase implements LoggableInputs {
    private final PIDMechanismBase leftWheel;
    private final PIDMechanismBase rightWheel;
    private final DigitalInput lightSensor;
    private boolean sensorActivated = false;
    private final DashTunableNumber indexTune;
    private double targetSpeed = 0.4;
    private boolean stopped = true;


    public IndexSubsystem() {
        String tuneName = INDEX_TUNING_ENABLED ? "Index: PID" : "";
        leftWheel = new PIDRotationalMechanism(
                INDEX_LEFT_MOTOR_ID,
                INDEX_PID,
                INDEX_KS,
                INDEX_KV,
                INDEX_KA,
                MotorModel.NEO_550,
                "LeftIndexer",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );

        rightWheel = new PIDRotationalMechanism(
                INDEX_RIGHT_MOTOR_ID,
                INDEX_PID,
                INDEX_KS,
                INDEX_KV,
                INDEX_KA,
                MotorModel.NEO_550,
                "RightIndexer",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS,
                true
        );

        leftWheel.setPIDControlSupplier(() -> false);
        rightWheel.setPIDControlSupplier(() -> false);

        if (INDEX_TUNING_ENABLED) {
            indexTune = new DashTunableNumber("Index: Speed", INDEX_SPEED);
            indexTune.addConsumer(this::setTargetSpeed);
        } else {
            indexTune = null;
        }

        lightSensor = new DigitalInput(INDEX_SENSOR_PORT);
    }

    public void setTargetSpeed(double rpm) { this.targetSpeed = rpm; }

    /** @return If the {@link IndexSubsystem} is at target. */
    public boolean atTarget() { return leftWheel.atTarget() && rightWheel.atTarget(); }

    @Override
    public void periodic() {
        leftWheel.update();
        rightWheel.update();

        if (indexTune != null && !stopped)
            indexTune.update();

        if (!RobotBase.isSimulation() && !Constants.isReplay())
            sensorActivated = lightSensor.get();
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     */
    public void start() {
        leftWheel.translateMotor(targetSpeed);
        rightWheel.translateMotor(targetSpeed);
        stopped = targetSpeed == 0;
    }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() { leftWheel.stop(); rightWheel.stop(); stopped = true; }

    public boolean hasNote() { return sensorActivated; }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("SensorActivated", sensorActivated);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.sensorActivated = table.get("SensorActivated", sensorActivated);
    }
}



