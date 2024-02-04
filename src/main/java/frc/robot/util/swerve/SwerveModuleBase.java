package frc.robot.util.swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.util.io.IOManager;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.config.ModuleSettings;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Control.*;

/**
 * A {@link SwerveModuleBase} is composed of two motors and two encoders:
 * a drive motor/encoder and a turn motor/encoder. The turn motor is
 * responsible for controlling the direction the drive motor faces, essentially
 * allowing the robot to move in any direction.
 * <p></p>
 * NOTE: This system is <b>ONLY</b> designed to work with brushless motors!
 *
 * @author Eric Gold
 */
public abstract class SwerveModuleBase implements LoggableInputs {
    private Rotation2d angleSetpoint = null;
    private Rotation2d turnRelativeOffset = null;
    private Double speedSetpoint = null;

    private final SimpleMotorFeedforward driveFF;
    private final PIDController driveController;
    private final PIDController turnController;

    private double drivePositionRad = 0.0;
    private double driveVelocityRadPerSec = 0.0;
    private double driveAppliedVolts = 0.0;
    private double driveCurrentAmps = 0.0;
    private Rotation2d turnPosition = new Rotation2d();
    private Rotation2d turnAbsolutePosition = new Rotation2d();
    private double turnVelocityRadPerSec = 0.0;
    private double turnCurrentAmps = 0.0;
    private double turnAppliedVolts = 0.0;

    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;

    private final Rotation2d absOffset;

    private RelativeEncoder driveEncoder;
    private RelativeEncoder turnEncoder;

    private final String name;

    public abstract Rotation2d getAbsolutePosition();

    /**
     * Creates a new {@link SwerveModuleBase} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name     The {@link String} name of the {@link SwerveModuleBase}.
     * @param settings The {@link ModuleSettings} of the {@link SwerveModuleBase}.
     */
    public SwerveModuleBase(String name, ModuleSettings settings) {
        this.driveController = PIDConstantsAK.generateController(CHASSIS_MODE.getDrivePID());
        this.turnController = PIDConstantsAK.generateController(CHASSIS_MODE.getTurnPID());
        this.driveMotor = new FRCSparkMax(settings.getDriveID(), kBrushless, MotorModel.NEO);
        this.turnMotor = new FRCSparkMax(settings.getTurnID(), kBrushless, MotorModel.NEO);
        this.absOffset = settings.getOffset();

        // TODO: make it a triple value like PIDConstantsAK if required.
        // TODO: make the values a constant
        this.driveFF = new SimpleMotorFeedforward(0.1, 0.13, 0);

        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();
        this.name = name;

        // Shorten the travel as much as possible for efficiency reasons.
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        // Enable voltage compensation to base calculations off a nominal 12V. Our robot *should* hopefully
        // exceed this figure during a match or while practice driving.
        driveMotor.enableVoltageCompensation(12.0);
        turnMotor.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);

        if (SWERVE_TUNING_ENABLED) {
            IOManager.initPIDTune("Swerve: Drive PID", driveController);
            IOManager.initPIDTune("Swerve: Turn PID", turnController);
        }
    }

    public void update() {
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        ////////////////////////////////////////////// Update all the inputs inside this method.
        drivePositionRad = CHASSIS_MODE
                .getDriveRatio()
                .getFollowerAngle(Rotation2d.fromRotations(driveEncoder.getPosition()))
                .getRadians();

        driveVelocityRadPerSec = CHASSIS_MODE
                .getDriveRatio()
                .getFollowerAngle(Rotation2d.fromRotations(driveEncoder.getVelocity()))
                .getRadians();

        driveAppliedVolts = driveMotor.getAppliedVoltage();
        driveCurrentAmps = driveMotor.getOutputCurrent();

        turnAbsolutePosition = getAbsolutePosition().minus(absOffset);
        turnPosition = CHASSIS_MODE
                .getTurnRatio()
                .getFollowerAngle(Rotation2d.fromRotations(turnEncoder.getPosition()));

        turnVelocityRadPerSec = CHASSIS_MODE
                .getTurnRatio()
                .getFollowerAngle(Rotation2d.fromRotations(turnEncoder.getVelocity()))
                .getRadians();

        turnAppliedVolts = turnMotor.getAppliedVoltage();
        turnCurrentAmps = turnMotor.getOutputCurrent();
        //////////////////////////////////////////////////////////////////////////////////////

        Logger.processInputs("Drive/Module" + name, this);
        Logger.recordOutput("Drive/Module" + name + "/RawTurnPos", getAbsolutePosition());

        if (turnRelativeOffset == null && turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = turnAbsolutePosition.minus(turnPosition);
        }

        // Run closed loop turn control
        if (angleSetpoint != null) {
            turnMotor.setVoltage(turnController.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));

            if (speedSetpoint != null) {
                double velocityRadPerSec = (speedSetpoint * Math.cos(turnController.getPositionError()))
                        / CHASSIS_MODE.getWheelRadius();

                driveMotor.setVoltage(
                        driveFF.calculate(velocityRadPerSec)
                                + driveController.calculate(driveVelocityRadPerSec, velocityRadPerSec)
                );
            }
        }
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return turnPosition.plus(turnRelativeOffset);
        }
    }

    /**
     * Returns the current drive velocity of the module in meters per second.
     */
    public double getVelocityMetersPerSec() {
        return driveVelocityRadPerSec * CHASSIS_MODE.getWheelRadius();
    }

    /**
     * Returns the drive velocity in radians/sec.
     */
    public double getCharacterizationVelocity() {
        return driveVelocityRadPerSec;
    }

    /**
     * Sets the state of the {@link SwerveModuleBase}.
     *
     * @param state The {@link SwerveModuleState} to use.
     */
    public SwerveModuleState setState(SwerveModuleState state) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        SwerveModuleState optimizedState = SwerveModuleState.optimize(state, getAngle());

        angleSetpoint = optimizedState.angle;
        speedSetpoint = optimizedState.speedMetersPerSecond;

        return optimizedState;
    }

    /**
     * Get the {@link SwerveModuleState} based on the drive motor's velocity
     * (meters/sec) and the turn encoder's angle.
     *
     * @return a new {@link SwerveModuleState}, representing the module's
     * current state, based on the module's drive motor velocity (m/s)
     * and the turn encoder's angle.
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getVelocityMetersPerSec(),
                getAngle()
        );
    }

    /**
     * Get the {@link SwerveModulePosition} based on the drive motor's
     * distance travelled (in meters), and turn encoder's angle. This
     * is required for {@link SwerveDriveOdometry} to work correctly.
     *
     * @return A {@link SwerveModulePosition}, representing the module's
     * current position, based on the module's drive motor distance and
     * the turn encoder's angle.
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDistance(),
                getAngle()
        );
    }

    public void runCharacterization(double volts) {
        angleSetpoint = new Rotation2d();

        if (!Constants.isReplay())
            driveMotor.setVoltage(volts);
        speedSetpoint = null;
    }

    /**
     * @return The name of the swerve module
     */
    public String getName() {
        return name;
    }

    /**
     * @return The total amount of meters the individual {@link SwerveModuleBase} has traveled.
     */
    public double getDistance() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return drivePositionRad * CHASSIS_MODE.getWheelRadius();
    }

    /**
     * Resets the relative drive encoder reading on the {@link SwerveModuleBase}.
     */
    public void reset() {
        driveEncoder.setPosition(0);
    }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("DrivePositionRad", this.drivePositionRad);
        table.put("DriveVelocityRadPerSec", this.driveVelocityRadPerSec);
        table.put("DriveAppliedVolts", this.driveAppliedVolts);
        table.put("DriveCurrentAmps", this.driveCurrentAmps);
        table.put("TurnAbsolutePosition", this.turnAbsolutePosition);
        table.put("TurnPosition", this.turnPosition);
        table.put("TurnVelocityRadPerSec", this.turnVelocityRadPerSec);
        table.put("TurnCurrentAmps", this.turnCurrentAmps);
        table.put("TurnAppliedVolts", this.turnAppliedVolts);
    }

    /**
     * Updates data based on a LogTable.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.drivePositionRad = table.get("DrivePositionRad", this.drivePositionRad);
        this.driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", this.driveVelocityRadPerSec);
        this.driveAppliedVolts = table.get("DriveAppliedVolts", this.driveAppliedVolts);
        this.driveCurrentAmps = table.get("DriveCurrentAmps", this.driveCurrentAmps);
        this.turnAbsolutePosition = table.get("TurnAbsolutePosition", this.turnAbsolutePosition);
        this.turnPosition = table.get("TurnPosition", this.turnPosition);
        this.turnVelocityRadPerSec = table.get("TurnVelocityRadPerSec", this.turnVelocityRadPerSec);
        this.turnCurrentAmps = table.get("TurnCurrentAmps", this.turnCurrentAmps);
        this.turnAppliedVolts = table.get("TurnAppliedVolts", this.turnAppliedVolts);
    }
}