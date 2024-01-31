package frc.robot.util.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.swerve.config.SwerveModuleIO;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.CHASSIS_BASE_RADIUS;
import static frc.robot.Constants.Chassis.CHASSIS_MODE;

public abstract class SwerveModuleSparkBase implements SwerveModuleIO {
    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;
    private final Rotation2d absOffset;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    public SwerveModuleSparkBase(int driveId, int turnId, double offsetRad) {
        this.driveMotor = new FRCSparkMax(driveId, kBrushless, MotorModel.NEO);
        this.turnMotor = new FRCSparkMax(turnId, kBrushless, MotorModel.NEO);

        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();
        this.absOffset = Rotation2d.fromRadians(offsetRad);

        driveMotor.enableVoltageCompensation(12.0);
        turnMotor.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);
    }

    /**
     * Resets all the encoders on the Swerve Module.
     */
    @Override
    public void reset() {
        driveEncoder.setPosition(0);
    }

    protected abstract double getAbsoluteRotations();

    /**
     * Updates the inputs of the {@link SwerveModuleIO}.
     *
     * @param inputs The {@link SwerveModuleIOInputs} to use.
     */
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        inputs.drivePositionRad = CHASSIS_MODE
                .getDriveRatio()
                .getFollowerAngle(Rotation2d.fromRotations(driveEncoder.getPosition()))
                .getRadians();

        inputs.driveVelocityRadPerSec = CHASSIS_MODE
                .getDriveRatio()
                .getFollowerAngle(Rotation2d.fromRotations(driveEncoder.getVelocity()))
                .getRadians();

        inputs.driveAppliedVolts = driveMotor.getAppliedVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();

        inputs.turnAbsolutePosition = Rotation2d.fromRotations(getAbsoluteRotations()).minus(absOffset);
        inputs.turnPosition = CHASSIS_MODE
                .getTurnRatio()
                .getFollowerAngle(Rotation2d.fromRotations(turnEncoder.getPosition()));

        inputs.turnVelocityRadPerSec = CHASSIS_MODE
                .getTurnRatio()
                .getFollowerAngle(Rotation2d.fromRotations(turnEncoder.getVelocity()))
                .getRadians();

        inputs.turnAppliedVolts = turnMotor.getAppliedVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
    }

    @Override
    public Rotation2d getRawAbsolutePosition() {
        return Rotation2d.fromRotations(getAbsoluteRotations());
    }

    @Override
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    @Override
    public void setTurnVoltage(double volts) {
        turnMotor.setVoltage(volts);
    }

    /**
     * @return The maximum <b>theoretical</b> stall current of this {@link IMotorModel} in <b>amperes.</b>
     */
    @Override
    public int getMaximumStallCurrent() {
        return driveMotor.getMaximumStallCurrent();
    }

    /**
     * @return The maximum <b>theoretical</b> free speed of this {@link IMotorModel} in <b>RPM.</b>
     */
    @Override
    public double getFreeSpeedRPM() {
        return driveMotor.getFreeSpeedRPM();
    }

    /**
     * @return The maximum <b>theoretical</b> stall torque of this {@link IMotorModel} in <b>newton-meters.</b>
     */
    @Override
    public double getStallTorqueNM() {
        return driveMotor.getStallTorqueNM();
    }

    /**
     * @param numMotors The number of motors to process.
     * @return The {@link DCMotor} instance used for simulation.
     */
    @Override
    public DCMotor getMotorInstance(int numMotors) {
        return driveMotor.getMotorInstance(numMotors);
    }
}
