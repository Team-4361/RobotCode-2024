package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static frc.robot.Constants.Indexer.*;

public class IndexSubsystem extends SubsystemBase implements LoggableInputs {
    private final PIDMechanismBase leftWheel;
    private final PIDMechanismBase rightWheel;
    private final ColorSensorV3 sensor;
    private final DashTunableNumber indexTune;
    private double targetRPM = INDEX_RPM;
    private boolean stopped = true;

    private double redValue = 0.0;
    private double greenValue = 0.0;
    private double blueValue = 0.0;

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
                RotationUnit.ROTATIONS
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
                RotationUnit.ROTATIONS
        );

        sensor = new ColorSensorV3(INDEX_SENSOR_PORT);

        if (INDEX_TUNING_ENABLED) {
            indexTune = new DashTunableNumber("Index: Speed", INDEX_RPM);
            indexTune.addConsumer(this::setTargetRPM);
        } else {
            indexTune = null;
        }
    }

    public void setTargetRPM(double rpm) { this.targetRPM = rpm; }

    /** @return If the {@link IndexSubsystem} is at target. */
    public boolean atTarget() { return leftWheel.atTarget() && rightWheel.atTarget(); }

    @Override
    public void periodic() {
        leftWheel.update();
        rightWheel.update();
        if (indexTune != null && !stopped)
            indexTune.update();

        if (!RobotBase.isSimulation()) {
            redValue = sensor.getRed();
            greenValue = sensor.getGreen();
            blueValue = sensor.getBlue();
        }

        Logger.processInputs("Index", this);
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     */
    public void start() {
        leftWheel.setTarget(targetRPM, true);
        rightWheel.setTarget(targetRPM, true);
        stopped = targetRPM == 0;
    }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() { leftWheel.stop(); rightWheel.stop(); stopped = true; }

    public boolean hasNote() {
        return ExtendedMath.inRange(redValue, RED_MINIMUM_TOLERANCE, RED_MAXIMUM_TOLERANCE) &&
                ExtendedMath.inRange(greenValue, GREEN_MINIMUM_TOLERANCE, GREEN_MAXIMUM_TOLERANCE) &&
                ExtendedMath.inRange(blueValue, BLUE_MINIMUM_TOLERANCE, BLUE_MAXIMUM_TOLERANCE);
    }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("Red", redValue);
        table.put("Green", greenValue);
        table.put("Blue", blueValue);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.redValue = table.get("Red", redValue);
        this.greenValue = table.get("Green", greenValue);
        this.blueValue = table.get("Blue", blueValue);
    }
}
