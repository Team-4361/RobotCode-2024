package frc.robot.subsystems;

import frc.robot.subsystems.base.BaseSubsystem;

import java.util.Map;

import static frc.robot.Constants.Indexer.*;
import static frc.robot.Constants.Systems.INDEX;

public class IndexSubsystem extends BaseSubsystem {
    public IndexSubsystem() {
        super(INDEX, Map.ofEntries(
                Map.entry(INDEX_LEFT_MOTOR_ID, true),
                Map.entry(INDEX_RIGHT_MOTOR_ID, false))
        );
        registerConstant("Speed", INDEX_SPEED);
    }

    public void setTargetSpeed(double speed) { setConstant("Speed", speed); }
    public double getTargetSpeed() { return getConstant("Speed"); }

    public void startNormal() { startAll(); }
    public void startReverse() { startAll(-getTargetSpeed());}
}


