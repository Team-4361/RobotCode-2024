package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDLinearMechanism;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;

import java.util.LinkedHashMap;

import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;
import static frc.robot.Constants.Climber.*;

public class ClimberSubsystem extends SubsystemBase {
    private final PIDLinearMechanism leftClimbMotor;
    private final PIDLinearMechanism rightClimbMotor;

    public ClimberSubsystem(LinkedHashMap<String, Double> map){
        leftClimbMotor = new PIDLinearMechanism(
                LEFT_CLIMB_MOTOR_ID,
                CLIMB_PID,
                CLIMB_KS,
                CLIMB_KV,
                CLIMB_KA,
                MotorModel.NEO,
                "LeftClimber",
                "",
                PIDLinearMechanism.DistanceUnit.INCHES,
                MAX_DISTANCE
        );

        rightClimbMotor = new PIDLinearMechanism(
                RIGHT_CLIMB_MOTOR_ID,
                CLIMB_PID,
                CLIMB_KS,
                CLIMB_KV,
                CLIMB_KA,
                MotorModel.NEO,
                "RightClimber",
                "",
                PIDLinearMechanism.DistanceUnit.INCHES,
                MAX_DISTANCE
        );
        leftClimbMotor.setPresets(map);
        rightClimbMotor.setPresets(map);

        leftClimbMotor.setTuneMode(CLIMBER_TUNING_ENABLED);
        rightClimbMotor.setTuneMode(CLIMBER_TUNING_ENABLED);
    }

    public boolean setPreset (String name){
        return leftClimbMotor.setPreset(name) && rightClimbMotor.setPreset(name);
    }

    public boolean setPreset (int idx) {
        return leftClimbMotor.setPreset(idx) && rightClimbMotor.setPreset(idx);
    }
}
