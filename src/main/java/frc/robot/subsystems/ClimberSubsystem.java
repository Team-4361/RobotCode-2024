package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.base.BaseSubsystem;
import frc.robot.util.motor.TimedDigitalInput;

import java.util.Map;

import static frc.robot.Constants.Climber.*;
import static frc.robot.Constants.Systems.CLIMBER;
import static java.util.Map.entry;

public class ClimberSubsystem extends BaseSubsystem {
    private final TimedDigitalInput leftSensor;
    private final TimedDigitalInput rightSensor;

    public enum MoveDirection { UP, DOWN }

    public ClimberSubsystem(){
        super(CLIMBER, CLIMBER_SPEED, Map.ofEntries(
                entry(CLIMBER_LEFT_ID, CLIMBER_LEFT_INVERTED),
                entry(CLIMBER_RIGHT_ID,CLIMBER_RIGHT_INVERTED))
        );
        this.leftSensor = new TimedDigitalInput(CLIMBER_LEFT_DIO);
        this.rightSensor = new TimedDigitalInput(CLIMBER_RIGHT_DIO);

        setDashUpdate(() -> {
            SmartDashboard.putBoolean("Climber: Left Down", leftSensor.get());
            SmartDashboard.putBoolean("Climber: Right Down", rightSensor.get());
        });
    }

    public long getLeftActivatedDuration()  { return leftSensor.getActivatedDuration();  }
    public long getRightActivatedDuration() { return rightSensor.getActivatedDuration(); }

    public void moveLeft(MoveDirection direction) {
        switch (direction) {
            case UP: setIndex(0, getTargetSpeed()); break;
            case DOWN: setIndex(0, -getTargetSpeed()); break;
        }
    }

    public void moveRight(MoveDirection direction) {
        switch (direction) {
            case UP: setIndex(1, getTargetSpeed()); break;
            case DOWN: setIndex(1, -getTargetSpeed()); break;
        }
    }

    public boolean isLeftRetracted() { return leftSensor.get(); }
    public boolean isRightRetracted() { return rightSensor.get(); }

    /**
     * This method is called periodically by the {@link CommandScheduler}. Useful for updating
     * subsystem-specific state that you don't want to offload to a {@link Command}. Teams should try
     * to be consistent within their own codebases about which responsibilities will be handled by
     * Commands, and which will be handled here.
     */
    @Override
    public void periodic() {
        super.periodic();
        leftSensor.update();
        rightSensor.update();
    }
}
