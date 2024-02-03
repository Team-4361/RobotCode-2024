package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDConstantsAK;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Control.SHOOTER_TUNING_ENABLED;
import static frc.robot.Constants.Shooter.*;

/**
 * This {@link ShooterSubsystem} is designed to enable the {@link Robot} to shoot. Physically, this
 * mechanism contains two motors which need to be driven opposite to each other.
 */
public class ShooterSubsystem extends SubsystemBase implements LoggableInputs {
    private final FRCSparkMax leftMotor;
    private final FRCSparkMax rightMotor;
    private double desiredRPM;

    // Loggable inputs used by AdvantageKit.
    private double leftVelocityRPM = 0.0;
    private double rightVelocityRPM = 0.0;
    private double leftAppliedVolts = 0.0;
    private double rightAppliedVolts = 0.0;
    private double leftCurrentAmps = 0.0;
    private double rightCurrentAmps = 0.0;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private final PIDController leftController;
    private final PIDController rightController;



    /**
     * Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values.
     */
    public ShooterSubsystem() {
        this.leftMotor = new FRCSparkMax(LEFT_SHOOTER_MOTOR_ID, kBrushless, MotorModel.NEO);
        this.rightMotor = new FRCSparkMax(RIGHT_SHOOTER_MOTOR_ID, kBrushless, MotorModel.NEO);
        this.leftEncoder = leftMotor.getEncoder();
        this.rightEncoder = rightMotor.getEncoder();

        this.leftController = PIDConstantsAK.generateController(SHOOT_PID);
        this.rightController = PIDConstantsAK.generateController(SHOOT_PID);

        if (SHOOTER_TUNING_ENABLED) {
            IOManager.initPIDTune("Shooter: PID", leftPID);
        }
    }

    /** @return If the {@link ShooterSubsystem} is at target. */
    public boolean atTarget() {
        // The first two conditions ensures the shooter must spin up!
        return ExtendedMath.inToleranceNotZero(desiredRPM, leftVelocityRPM, 100) &&
                ExtendedMath.inToleranceNotZero(-desiredRPM, rightVelocityRPM, 100);
    }

    @Override
    public void periodic() {
        //////////////////////////////////////////////////////////////////////////// Update all AK inputs
        leftEncoder = leftMotor.getEncoder();
        rightEncoder = rightMotor.getEncoder();

        leftVelocityRPM = leftEncoder.getVelocity();
        rightVelocityRPM = rightEncoder.getVelocity();

        leftAppliedVolts = leftMotor.getAppliedVoltage();
        rightAppliedVolts = rightMotor.getAppliedVoltage();

        leftCurrentAmps = leftMotor.getOutputCurrent();
        rightCurrentAmps = rightMotor.getOutputCurrent();
        ////////////////////////////////////////////////////////////////////////////////////////////////

        Logger.processInputs("Shooter", this);

        // Attempt to move the motors into their desired targets.
        leftMotor.set(kVelocity, desiredRPM);
        rightMotor.set(kVelocity, -desiredRPM);

        Logger.recordOutput("Shooter/TargetReached", atTarget());
    }

    /**
     * Sets the target of the {@link ShooterSubsystem}.
     * @param rpm The desired RPM.
     */
    public void setTarget(double rpm) { this.desiredRPM = rpm; }

    /** Stops the {@link ShooterSubsystem} from spinning. */
    public void stop() { setTarget(0); }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        this.leftVelocityRPM   = table.get("LeftVelocityRadPerSec", this.leftVelocityRPM);
        this.rightVelocityRPM  = table.get("RightVelocityRadPerSec", this.rightVelocityRPM);
        this.leftAppliedVolts  = table.get("LeftAppliedVolts", this.leftAppliedVolts);
        this.rightAppliedVolts = table.get("RightAppliedVolts", this.rightAppliedVolts);
        this.leftCurrentAmps   = table.get("LeftCurrentAmps", this.leftCurrentAmps);
        this.rightCurrentAmps  = table.get("RightCurrentAmps", this.rightCurrentAmps);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        table.put("LeftVelocityRadPerSec", this.leftVelocityRPM);
        table.put("RightVelocityRadPerSec", this.rightVelocityRPM);
        table.put("LeftAppliedVolts", this.leftAppliedVolts);
        table.put("RightAppliedVolts", this.rightAppliedVolts);
        table.put("LeftCurrentAmps", this.leftCurrentAmps);
        table.put("RightCurrentAmps", this.rightCurrentAmps);
    }
}
