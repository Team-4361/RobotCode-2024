package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
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

import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.*;

public class IndexSubsystem extends SubsystemBase {
    private final PIDMechanismBase leftWheel;
    private final PIDMechanismBase rightWheel;
    private final DashTunableNumber indexTune;
    private double targetRPM = 5000;
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
}



