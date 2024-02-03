package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.pid.PIDWheelModule;

import static frc.robot.Constants.Indexer.*;

public class IndexSubsystem extends SubsystemBase {
    private final PIDWheelModule leftWheel;
    private final PIDWheelModule rightWheel;
    private final ColorSensorV3 sensor;

    public IndexSubsystem() {
        String tuneName = INDEX_TUNING_ENABLED ? "Index: PID" : "";
        leftWheel = new PIDWheelModule(
                INDEX_LEFT_MOTOR_ID,
                INDEX_PID,
                INDEX_KS,
                INDEX_KV,
                INDEX_KA,
                "LeftIndexer",
                tuneName
        );

        rightWheel = new PIDWheelModule(
                INDEX_RIGHT_MOTOR_ID,
                INDEX_PID,
                INDEX_KS,
                INDEX_KV,
                INDEX_KA,
                "RightIndexer",
                tuneName
        );

        sensor = new ColorSensorV3(INDEX_SENSOR_PORT);
    }

    /** @return If the {@link IndexSubsystem} is at target. */
    public boolean atTarget() { return leftWheel.atTarget() && rightWheel.atTarget(); }

    @Override
    public void periodic() {
        leftWheel.update();
        rightWheel.update();
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) {
        leftWheel.setTarget(rpm);
        rightWheel.setTarget(-rpm);
    }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() { setTarget(0); }

    public boolean hasNote() {
        return ExtendedMath.inRange(sensor.getRed(), RED_MINIMUM_TOLERANCE, RED_MAXIMUM_TOLERANCE) &&
                ExtendedMath.inRange(sensor.getGreen(), GREEN_MINIMUM_TOLERANCE, GREEN_MAXIMUM_TOLERANCE) &&
                ExtendedMath.inRange(sensor.getBlue(), BLUE_MINIMUM_TOLERANCE, BLUE_MAXIMUM_TOLERANCE);
    }
}
