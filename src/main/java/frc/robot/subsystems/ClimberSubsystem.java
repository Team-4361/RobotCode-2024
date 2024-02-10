package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDLinearMechanism;

import java.util.LinkedHashMap;

import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;
import static frc.robot.Constants.Climber.*;

public class ClimberSubsystem extends SubsystemBase {
    private final PIDLinearMechanism leftClimber;
    private final PIDLinearMechanism rightClimber;

    public ClimberSubsystem(){
        leftClimber = new PIDLinearMechanism(
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

        rightClimber = new PIDLinearMechanism(
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
        leftClimber.setTuneMode(CLIMBER_TUNING_ENABLED);
        rightClimber.setTuneMode(CLIMBER_TUNING_ENABLED);
    }

    public PIDLinearMechanism getLeftClimber() { return this.leftClimber; }
    public PIDLinearMechanism getRightClimber() { return this.rightClimber; }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        leftClimber.update();
        rightClimber.update();
    }
}
