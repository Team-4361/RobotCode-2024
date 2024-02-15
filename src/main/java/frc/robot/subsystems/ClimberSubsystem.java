package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.PIDLinearMechanism;

import java.util.LinkedHashMap;

import static frc.robot.Constants.Debug.CLIMBER_TUNING_ENABLED;
import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Debug.INDEX_TUNING_ENABLED;
import static frc.robot.Constants.Indexer.INDEX_SPEED;

public class ClimberSubsystem extends SubsystemBase {
    private final PIDLinearMechanism leftClimber;
    private final PIDLinearMechanism rightClimber;
    private final DigitalInput magnetSensor;
    private boolean magnetSensorActivated = false;
    private final DashTunableNumber climbTune;
    private boolean stopped = false;
    // have to figure out speed later (do we even need?)
    private double climberSpeed = 0.25;
    // This {@link ClimberSubsystem} is used to control the {@link Robot}'s climbing system. Physically, the climber is
    // made of two climbers in a box powered individually by a NEO.

    public ClimberSubsystem() {
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
        magnetSensor = new DigitalInput(MAGNET_SENSOR_PORT);

        if (CLIMBER_TUNING_ENABLED) {
            climbTune = new DashTunableNumber("Climbers: Speed", climberSpeed);
        } else {
            climbTune = null;
        }
    }

    public void setTargetSpeed(double rpm) {
        this.climberSpeed = rpm;
    }

    public PIDLinearMechanism getLeftClimber() {
        return this.leftClimber;
    }

    public PIDLinearMechanism getRightClimber() {
        return this.rightClimber;
    }

    public void start() {
        leftClimber.translateMotor(climberSpeed);
        rightClimber.translateMotor(climberSpeed);
        stopped = (climberSpeed == 0);
    }

    public void stop() {
        leftClimber.stop();
        rightClimber.stop();
        stopped = true;
    }

    public boolean climberDown() {
        return magnetSensorActivated;
    }
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

        if (climbTune != null && !stopped)
            climbTune.update();

        if (!RobotBase.isSimulation() && !Constants.isReplay())
            magnetSensorActivated = magnetSensor.get();
    }


}
