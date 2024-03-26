package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.pid.TunableNumber;

import java.util.Map;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.*;

public class IndexSubsystem extends BaseSubsystem {
    public IndexSubsystem() {
        super(
                "Index",
                INDEX_SPEED,
                INDEX_TUNING_ENABLED,
                Map.ofEntries(
                        Map.entry(INDEX_LEFT_MOTOR_ID, true),
                        Map.entry(INDEX_RIGHT_MOTOR_ID, false)
                )
        );
    }

    public void startNormal() { startAll(); }
    public void startReverse() { startAll(-getTargetSpeed());}
}


