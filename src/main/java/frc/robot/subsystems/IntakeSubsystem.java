package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GearRatio;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDMechanismBase;
import frc.robot.util.pid.PIDRotationalMechanism;
import frc.robot.util.pid.PIDRotationalMechanism.RotationUnit;

import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
    private final PIDMechanismBase intakeWheel;

    public IntakeSubsystem() {
        String tuneName = INTAKE_TUNING_ENABLED ? "Intake: PID" : "";
        intakeWheel = new PIDRotationalMechanism(
                INTAKE_MOTOR_ID,
                INTAKE_PID,
                INTAKE_KS,
                INTAKE_KV,
                INTAKE_KA,
                MotorModel.NEO,
                "Intake",
                tuneName,
                GearRatio.DIRECT_DRIVE,
                RotationUnit.ROTATIONS
        );
    }

    /** @return If the {@link IndexSubsystem} is at target. */
    public boolean atTarget() { return intakeWheel.atTarget(); }

    @Override public void periodic() { intakeWheel.update(); }

    /**
     * Sets the target of the {@link IndexSubsystem}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) { intakeWheel.setTarget(INTAKE_INVERTED ? -rpm: rpm, true); }

    /** Stops the {@link IndexSubsystem} from spinning. */
    public void stop() { setTarget(0); }
}
