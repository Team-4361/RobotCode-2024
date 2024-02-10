package frc.robot.util.swerve;

import com.revrobotics.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.util.io.IOManager;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.swerve.config.ModuleSettings;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import static com.revrobotics.CANSparkBase.ControlType.kPosition;
import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.SWERVE_TUNING_ENABLED;

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
    private Rotation2d lastAngle;

    private final SimpleMotorFeedforward driveFF;
    private final SparkPIDController driveController;
    private final SparkPIDController turnController;

    private double drivePositionMeters = 0.0;
    private double driveVelocityMPS = 0.0;
    private double driveAppliedVolts = 0.0;
    private double driveCurrentAmps = 0.0;
    private Rotation2d turnPosition = new Rotation2d();
    private Rotation2d turnAbsolutePosition = new Rotation2d();
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
        this.driveMotor = new FRCSparkMax(settings.getDriveID(), kBrushless, MotorModel.NEO);
        this.turnMotor = new FRCSparkMax(settings.getTurnID(), kBrushless, MotorModel.NEO);
        this.driveController = driveMotor.getPIDController();
        this.turnController = turnMotor.getPIDController();
        this.absOffset = Rotation2d.fromRadians(settings.getOffsetDegrees());
        this.lastAngle = new Rotation2d();

        CHASSIS_MODE.getDrivePID().initController(driveController);
        CHASSIS_MODE.getTurnPID().initController(turnController);

        // TODO: Will this cause problems?
        this.driveFF = CHASSIS_MODE.getFeedForward();
        this.driveEncoder = driveMotor.getEncoder();
        this.turnEncoder = turnMotor.getEncoder();
        this.name = name;

        turnEncoder.setPositionConversionFactor(TURN_POSITION_FACTOR);
        driveEncoder.setPositionConversionFactor(DRIVE_POSITION_FACTOR);
        driveEncoder.setVelocityConversionFactor(DRIVE_VELOCITY_FACTOR);

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);

        if (SWERVE_TUNING_ENABLED) {
            IOManager.initPIDTune("Swerve: Drive PID", driveController);
            IOManager.initPIDTune("Swerve: Turn PID", turnController);
        }
        Timer.delay(1);

        turnEncoder.setPosition(getAbsolutePosition().minus(absOffset).getDegrees());
    }

    public void update() {
        driveEncoder = driveMotor.getEncoder();
        turnEncoder = turnMotor.getEncoder();

        ////////////////////////////////////////////// Update all the inputs inside this method.
        drivePositionMeters = driveEncoder.getPosition();
        driveVelocityMPS = driveEncoder.getVelocity();

        driveAppliedVolts = driveMotor.getAppliedVoltage();
        driveCurrentAmps = driveMotor.getOutputCurrent();

        turnAbsolutePosition = getAbsolutePosition().minus(absOffset);
        turnPosition = Rotation2d.fromDegrees(turnEncoder.getPosition());

        turnAppliedVolts = turnMotor.getAppliedVoltage();
        turnCurrentAmps = turnMotor.getOutputCurrent();
        //////////////////////////////////////////////////////////////////////////////////////

        Logger.processInputs("Drive/Module" + name, this);
    }

    public SwerveModuleState setState(SwerveModuleState desiredState) {
        desiredState = GlobalUtils.optimize(desiredState, getState().angle);

        // Prevent rotating module if speed is less than 1% to prevent jerky movement.
        Rotation2d angle =
                (Math.abs(desiredState.speedMetersPerSecond) <= (CHASSIS_MODE.getMaxSpeed() * 0.01))
                        ? lastAngle
                        : desiredState.angle;

        turnController.setReference(angle.getDegrees(), kPosition);
        lastAngle = angle;

        driveController.setReference(
                desiredState.speedMetersPerSecond,
                kVelocity,
                0,
                driveFF.calculate(desiredState.speedMetersPerSecond)
        );

        return desiredState;
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
                driveVelocityMPS,
                turnPosition
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
                drivePositionMeters,
                turnAbsolutePosition
        );
    }

    /** @return The name of the {@link SwerveModuleBase}. */
    public String getName() { return name; }

    /**
     * Updates a LogTable with the data to log.
     *
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("DrivePositionMeters", this.drivePositionMeters);
        table.put("DriveVelocityMPS", this.driveVelocityMPS);
        table.put("DriveAppliedVolts", this.driveAppliedVolts);
        table.put("DriveCurrentAmps", this.driveCurrentAmps);
        table.put("TurnAbsolutePosition", this.turnAbsolutePosition);
        table.put("TurnPosition", this.turnPosition);
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
        this.drivePositionMeters = table.get("DrivePositionRad", this.drivePositionMeters);
        this.driveVelocityMPS = table.get("DriveVelocityRadPerSec", this.driveVelocityMPS);
        this.driveAppliedVolts = table.get("DriveAppliedVolts", this.driveAppliedVolts);
        this.driveCurrentAmps = table.get("DriveCurrentAmps", this.driveCurrentAmps);
        this.turnAbsolutePosition = table.get("TurnAbsolutePosition", this.turnAbsolutePosition);
        this.turnPosition = table.get("TurnPosition", this.turnPosition);
        this.turnCurrentAmps = table.get("TurnCurrentAmps", this.turnCurrentAmps);
        this.turnAppliedVolts = table.get("TurnAppliedVolts", this.turnAppliedVolts);
    }
}