package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;

import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;
import static frc.robot.Constants.Climber.*;

public class ClimberSubsystem extends SubsystemBase {
    private final PIDMechanismBase leftClimbMotor;
    private final PIDMechanismBase rightClimbMotor;
    private final DashTunableNumber climbTune;
    //private final targetRPM = ;
    private final boolean stopped = true;

    public ClimberSubsystem (){
        String tuneName = CLIMBER_TUNING_ENABLED ? "Climber: PID" : "";
        leftClimbMotor = new PIDRotationalMechanism(
                LEFT_CLIMB_MOTOR_ID,
                CLIMB_PID,
                CLIMB_KS,
                CLIMB_KV,
                CLIMB_KA,
                MotorModel.NEO,
                "LeftClimber",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                PIDRotationalMechanism.RotationUnit.ROTATIONS
        );

    }


   // **@Override
    protected double getCurrentPosition(double motorRotations) {
        return 0;
    }







}
