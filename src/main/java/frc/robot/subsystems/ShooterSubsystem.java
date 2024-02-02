package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.math.ExtendedMath;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.SHOOTER_MOTOR_1_ID;
import static frc.robot.Constants.SHOOTER_MOTOR_2_ID;
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

    /**
     * Constructs a new {@link ShooterSubsystem} using all <code>CONSTANTS</code> values.
     */
    public ShooterSubsystem() {
        this.leftMotor = new FRCSparkMax(LEFT_SHOOTER_MOTOR_ID, kBrushless, MotorModel.NEO);
        this.rightMotor = new FRCSparkMax(RIGHT_SHOOTER_MOTOR_ID, kBrushless, MotorModel.NEO);
        this.leftEncoder = leftMotor.getEncoder();
        this.rightEncoder = rightMotor.getEncoder();

        SparkPIDController leftPID = leftMotor.getPIDController();
        SparkPIDController rightPID = rightMotor.getPIDController();

        SHOOT_PID.initController(leftPID::setP, leftPID::setI, leftPID::setD);
        SHOOT_PID.initController(rightPID::setP, rightPID::setI, rightPID::setD);

        leftPID.setFF(SHOOT_FEED_FWD);
        rightPID.setFF(SHOOT_FEED_FWD);

        leftMotor.enableVoltageCompensation(12);
        rightMotor.enableVoltageCompensation(12);
    }

    public boolean atTarget() {
        // The first two conditions ensures the shooter must spin up!
        return !ExtendedMath.inTolerance(0, leftVelocityRPM, 50) &&
                !ExtendedMath.inTolerance(0, rightVelocityRPM, 50) &&
                ExtendedMath.inTolerance(desiredRPM, leftVelocityRPM, 100) &&
                ExtendedMath.inTolerance(-desiredRPM, rightVelocityRPM, 100);
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

    public void setTarget(double rpm) { this.desiredRPM = rpm; }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        this.leftVelocityRPM = table.get("LeftVelocityRadPerSec", this.leftVelocityRPM);
        this.rightVelocityRPM = table.get("RightVelocityRadPerSec", this.rightVelocityRPM);
        this.leftAppliedVolts       = table.get("LeftAppliedVolts", this.leftAppliedVolts);
        this.rightAppliedVolts      = table.get("RightAppliedVolts", this.rightAppliedVolts);
        this.leftCurrentAmps        = table.get("LeftCurrentAmps", this.leftCurrentAmps);
        this.rightCurrentAmps       = table.get("RightCurrentAmps", this.rightCurrentAmps);
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
