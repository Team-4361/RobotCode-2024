package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDWheelModule;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final PIDWheelModule intakeWheel;

    public IntakeSubsystem() {
        String tuneName = INTAKE_TUNING_ENABLED ? "Intake: PID" : "";
        intakeWheel = new PIDWheelModule(
                INTAKE_MOTOR_ID,
                INTAKE_PID,
                INTAKE_KS,
                INTAKE_KV,
                INTAKE_KA,
                MotorModel.NEO,
                "Intake",
                tuneName
        );
    }

    /** @return If the {@link IndexSubsystem} is at target. */
    public boolean atTarget() { return intakeWheel.atTarget(); }

    @Override
    public void periodic() {
        intakeWheel.update();
    }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) {
        intakeWheel.setTarget(INTAKE_INVERTED ? -rpm: rpm);
    }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() { setTarget(0); }
}
