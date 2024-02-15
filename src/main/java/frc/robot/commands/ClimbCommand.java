package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;

public class ClimbCommand extends Command {

    public ClimbCommand(){addRequirements(Robot.climber);}

    @Override
    public void execute() {
        Robot.climber.start();
    }

    @Override
    public void end(boolean magnetSensorActivated) {
        Robot.climber.stop();
    }




}
