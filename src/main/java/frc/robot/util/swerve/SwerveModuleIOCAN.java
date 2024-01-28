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
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.swerve.config.SwerveModuleIO;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class SwerveModuleIOCAN implements SwerveModuleIO {
    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;
    private final Rotation2d absOffset;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private final StatusSignal<Double> absEncoderPosition;

    public SwerveModuleIOCAN(int driveId, int turnId, int dioPort, double offsetRad) {
        this.driveMotor = new FRCSparkMax(driveId, kBrushless, MotorModel.NEO);
        this.turnMotor = new FRCSparkMax(turnId, kBrushless, MotorModel.NEO);

        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();

        driveMotor.setCANTimeout(250);
        turnMotor.setCANTimeout(250);

        driveMotor.enableVoltageCompensation(12.0);
        turnMotor.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        driveEncoder.setMeasurementPeriod(10);
        driveEncoder.setAverageDepth(2);

        turnEncoder.setPosition(0.0);
        turnEncoder.setMeasurementPeriod(10);
        turnEncoder.setAverageDepth(2);

        driveMotor.setIdleMode(kBrake);
        turnMotor.setIdleMode(kBrake);

        driveMotor.setSmartCurrentLimit(40);
        turnMotor.setSmartCurrentLimit(20);

        driveMotor.setCANTimeout(0);
        turnMotor.setCANTimeout(0);

        ///////////////////////////////////////////////////////////////// Encoder Configuration
        CANcoderConfigurator config;
        try (CANcoder absEncoder = new CANcoder(dioPort)) {
            this.absOffset = Rotation2d.fromRotations(offsetRad);
            this.absEncoderPosition = absEncoder.getAbsolutePosition();

            config = absEncoder.getConfigurator();
        }

        config.apply(new CANcoderConfiguration());
        MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
        config.refresh(magnetConfig);
        config.apply(magnetConfig
                .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));
    }

    /**
     * Resets all the encoders on the Swerve Module.
     */
    @Override
    public void reset() {
        driveEncoder.setPosition(0);
    }

    /**
     * Updates the inputs of the {@link SwerveModuleIO}.
     *
     * @param inputs The {@link SwerveModuleIOInputs} to use.
     */
    @Override
    public void updateInputs(SwerveModuleIOInputs inputs) {
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        inputs.drivePositionRad = Units.rotationsToRadians(driveEncoder.getPosition())
                / driveMotor.getRatio().getDivisor();
        inputs.driveVelocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(driveEncoder.getVelocity())
                / driveMotor.getRatio().getDivisor();

        inputs.driveAppliedVolts = driveMotor.getAppliedVoltage();
        inputs.driveCurrentAmps = driveMotor.getOutputCurrent();
        inputs.turnAbsolutePosition = Rotation2d.fromRotations(
                absEncoderPosition.refresh().getValueAsDouble())
                .minus(absOffset);
        inputs.turnPosition = Rotation2d.fromRotations(turnEncoder.getPosition() / turnMotor.getRatio().getDivisor());
        inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnEncoder.getVelocity())
                / turnMotor.getRatio().getDivisor();

        inputs.turnAppliedVolts = turnMotor.getAppliedVoltage();
        inputs.turnCurrentAmps = turnMotor.getOutputCurrent();
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
