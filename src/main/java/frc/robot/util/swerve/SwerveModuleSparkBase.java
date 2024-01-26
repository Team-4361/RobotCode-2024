package frc.robot.util.swerve;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.*;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.swerve.config.SwerveModuleIO;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public abstract class SwerveModuleSparkBase implements SwerveModuleIO {
    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;

    public SwerveModuleSparkBase(int driveId, int turnId) {
        this.driveMotor = new FRCSparkMax(driveId, kBrushless, MotorModel.NEO);
        this.turnMotor = new FRCSparkMax(turnId, kBrushless, MotorModel.NEO);
        this.driveEncoder = driveMotor.getEncoder();

        driveMotor.enableVoltageCompensation(12);
        driveMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        turnMotor.setIdleMode(CANSparkBase.IdleMode.kBrake);
        driveMotor.setSmartCurrentLimit(40);
        turnMotor.setSmartCurrentLimit(20);
    }

    /**
     * Resets all the encoders on the Swerve Module.
     */
    @Override
    public void reset() {
        driveEncoder.setPosition(0);
    }

    protected abstract double getTurnRotation();

    /**
     * Updates the inputs of the {@link SwerveModuleIO}.
     *
     * @param inputs The {@link SwerveModuleIOInputs} to use.
     */
    @Override
    public void updateInputs(SwerveModuleIO.SwerveModuleIOInputs inputs) {
        inputs.drivePower = driveMotor.get();
        inputs.turnPower = turnMotor.get();
        inputs.driveRotations = driveEncoder.getPosition();
        inputs.turnRotations = getTurnRotation();
        inputs.driveRPM = driveEncoder.getVelocity();
    }

    /**
     * @return The {@link SparkPIDController} used for driving.
     */
    @Override
    public SparkPIDController getDrivePIDController() {
        return driveMotor.getPIDController();
    }

    /**
     * Sets the integrated PID constants to the specific values.
     *
     * @param pid The {@link PIDConstants} to use.
     */
    @Override
    public void setPID(PIDConstants pid) {
        SparkPIDController controller = driveMotor.getPIDController();
        controller.setP(pid.kP);
        controller.setI(pid.kI);
        controller.setD(pid.kD);
    }

    /**
     * Sets the drive power of the {@link SwerveModuleIO} in open-loop fashion.
     *
     * @param power The power from -1.0 to +1.0
     */
    @Override
    public void driveOpenLoop(double power) {
        driveMotor.set(power);
    }

    /**
     * Sets the drive RPM of the {@link SwerveModuleIO} in closed-loop operation.
     *
     * @param motorRPM The set-point drive motor RPM.
     */
    @Override
    public void driveClosedLoop(double motorRPM) {
        getDrivePIDController().setReference(motorRPM, CANSparkBase.ControlType.kVelocity);
    }

    /**
     * Sets the turn power of the {@link SwerveModuleIO} in open-loop fashion.
     *
     * @param power The power from -1.0 to +1.0
     */
    @Override
    public void setTurnPower(double power) {
        turnMotor.set(power);
    }

    /**
     * @return The maximum <b>theoretical</b> stall current of this {@link IMotorModel} in <b>amperes.</b>
     */
    @Override public int getMaximumStallCurrent() { return driveMotor.getMaximumStallCurrent(); }

    /**
     * @return The maximum <b>theoretical</b> free speed of this {@link IMotorModel} in <b>RPM.</b>
     */
    @Override public double getFreeSpeedRPM() { return driveMotor.getFreeSpeedRPM(); }

    /**
     * @return The maximum <b>theoretical</b> stall torque of this {@link IMotorModel} in <b>newton-meters.</b>
     */
    @Override public double getStallTorqueNM() { return driveMotor.getStallTorqueNM(); }
}
