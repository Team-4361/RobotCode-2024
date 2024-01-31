package frc.robot.util.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.util.io.AlertType;
import frc.robot.util.io.IOManager;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunablePID;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.config.ModuleSettings;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import java.util.function.Supplier;

import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.LooperConfig.STRING_DASHBOARD_NAME;

/**
 * A {@link SwerveModule} is composed of two motors and two encoders:
 * a drive motor/encoder and a turn motor/encoder. The turn motor is
 * responsible for controlling the direction the drive motor faces, essentially
 * allowing the robot to move in any direction.
 * <p></p>
 * NOTE: This system is <b>ONLY</b> designed to work with brushless motors!
 *
 * @author Eric Gold
 */
public class SwerveModule implements LoggableInputs {
    private Rotation2d angleSetpoint = null;
    private Rotation2d turnRelativeOffset = null;
    private Double speedSetpoint = null;

    private final SimpleMotorFeedforward driveFF;
    private final PIDController driveController;
    private final PIDController turnController;

    private static DashTunablePID driveTune = null;
    private static DashTunablePID turnTune = null;

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
    private Supplier<Rotation2d> absSupplier;

    private final String name;

    //public static final DashTunablePID driveTune = new DashTunablePID("Drive PID", DRIVE_PID_CONFIG);
    //public static final DashTunablePID steerTune = new DashTunablePID("Steer PID", TURN_PID_CONFIG);
    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name            The {@link String} name of the {@link SwerveModule}.
     * @param settings        The {@link ModuleSettings} of the {@link SwerveModule}.
     */
    public SwerveModule(String name, ModuleSettings settings) {
        this.driveController = PIDConstantsAK.generateController(CHASSIS_MODE.getDrivePID());
        this.turnController  = PIDConstantsAK.generateController(CHASSIS_MODE.getTurnPID());
        this.driveMotor      = new FRCSparkMax(settings.getDriveID(), kBrushless, MotorModel.NEO);
        this.turnMotor       = new FRCSparkMax(settings.getTurnID(),  kBrushless, MotorModel.NEO);
        this.absSupplier     = () -> Rotation2d.fromRotations(0);
        this.absOffset       = settings.getOffset();

        this.driveFF         = new SimpleMotorFeedforward(0.1, 0.13, 0);

        this.driveEncoder    = driveMotor.getEncoder();
        this.turnEncoder     = turnMotor.getEncoder();
        this.name = name;

        // Shorten the travel as much as possible for efficiency reasons.
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        // Enable voltage compensation to base calculations off a nominal 12V. Our robot *should* hopefully
        // exceed this figure during a match or while practice driving.
        driveMotor.enableVoltageCompensation(12.0);
        turnMotor.enableVoltageCompensation(12.0);

        driveEncoder.setPosition(0.0);
        turnEncoder.setPosition(0.0);

        // Create the absolute encoder instance based upon the CHASSIS_MODE.
        if (CHASSIS_MODE.usingMagEncoders()) {
            try (DutyCycleEncoder encoder = new DutyCycleEncoder(settings.getEncoderID())) {
                absSupplier = () -> Rotation2d.fromRotations(encoder.getAbsolutePosition());
            } catch (Exception ex) {
                IOManager.getAlert("Module " + name + ": Failed to create ABS encoder!", AlertType.ERROR)
                        .setPersistent(true)
                        .setEnabled(true);
            }
        } else {
            CANcoderConfigurator config;
            StatusSignal<Double> signal;
            try (CANcoder encoder = new CANcoder(settings.getEncoderID())) {
                signal = encoder.getAbsolutePosition();
                config = encoder.getConfigurator();

                config.apply(new CANcoderConfiguration());
                MagnetSensorConfigs magnetConfig = new MagnetSensorConfigs();
                config.refresh(magnetConfig);
                config.apply(magnetConfig
                        .withAbsoluteSensorRange(AbsoluteSensorRangeValue.Unsigned_0To1)
                        .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));

                absSupplier = () -> Rotation2d.fromRotations(signal.refresh().getValueAsDouble());
            } catch (Exception ex) {
                IOManager.getAlert("Module " + name + ": Failed to create ABS encoder!", AlertType.ERROR)
                        .setPersistent(true)
                        .setEnabled(true);
            }
        }

        if (SWERVE_TUNING_ENABLED) {
            // PID tuning is enabled.
            if (driveTune == null || turnTune == null) {
                driveTune = new DashTunablePID("Swerve: Drive PID", CHASSIS_MODE.getDrivePID());
                turnTune = new DashTunablePID("Swerve: Turn PID", CHASSIS_MODE.getTurnPID());

                IOManager.addPeriodicIfExists(STRING_DASHBOARD_NAME, () -> {
                    driveTune.update();
                    turnTune.update();
                });
            }

            driveTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
            turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);
        } else {
            driveTune = null;
            turnTune = null;
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

        turnAbsolutePosition = absSupplier.get().minus(absOffset);
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
        Logger.recordOutput("Drive/Module" + name + "/RawTurnPos", absSupplier.get());

        if (turnRelativeOffset == null && turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = turnAbsolutePosition.minus(turnPosition);
        }

        // Run closed loop turn control
        if (angleSetpoint != null) {
            if (!Constants.isReplay()) {
                turnMotor.setVoltage(
                        turnController.calculate(getAngle().getRadians(), angleSetpoint.getRadians()));
            }

            if (speedSetpoint != null) {
                double velocityRadPerSec = (speedSetpoint * Math.cos(turnController.getPositionError()))
                        / CHASSIS_MODE.getWheelRadius();

                // *** IMPORTANT: Do not actually set the motor values during replay! ***
                if (!Constants.isReplay()) {
                    driveMotor.setVoltage(
                            driveFF.calculate(velocityRadPerSec)
                                    + driveController.calculate(driveVelocityRadPerSec, velocityRadPerSec)
                    );
                }
            }
        }
    }

    public Rotation2d getAngle() {
        //return inputs.turnPosition;
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            return turnPosition.plus(turnRelativeOffset);
        }
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() { return driveVelocityRadPerSec * CHASSIS_MODE.getWheelRadius(); }

    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity() { return driveVelocityRadPerSec; }

    /**
     * Sets the state of the {@link SwerveModule}.
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

    /*
    public void updateDashboard() {

        if (DEBUG_ENABLED) {
            // Debug mode is enabled; place individual numbers on the Dashboard.
            String driveVelocity = name + ": motor rpm";
            String drivePower = name + ": pow";
            String turnPower = name + ": turn pow";
            String turnPosition = name + ": turn rad";
            String drivePosition = name + ": distance";

            SmartDashboard.putNumber(driveVelocity, driveEncoder.getVelocity());
            SmartDashboard.putNumber(turnPower, turnMotor.get());
            SmartDashboard.putNumber(turnPosition, getTurnAngle());
            SmartDashboard.putNumber(drivePower, driveMotor.get());
            SmartDashboard.putNumber(drivePosition, getDistance());
        } else {
            // Debug mode is disabled; only display bare-minimum entries on the Dashboard.
            String output = driveMotor.get() * 100 + "%|" + driveEncoder.getVelocity() + " rpm|" +
                    Units.radiansToDegrees(getTurnAngle()) + " deg";

            SmartDashboard.putString(name + " Module", output);
        }

        // 1-20-24: The "DashTuneablePID" automatically enforces a delay between updates.
        if (SWERVE_TUNING_ENABLED && driveTune != null && turnTune != null) {
            driveTune.update();
            turnTune.update();
        }
    }
     */

    /**
     * @return The total amount of meters the individual {@link SwerveModule} has traveled.
     */
    public double getDistance() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return drivePositionRad * CHASSIS_MODE.getWheelRadius();
    }

    /** Resets the relative drive encoder reading on the {@link SwerveModule}. */
    public void reset() { driveEncoder.setPosition(0); }

    /**
     * Updates a LogTable with the data to log.
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void toLog(LogTable table) {
        table.put("DrivePositionRad",       this.drivePositionRad);
        table.put("DriveVelocityRadPerSec", this.driveVelocityRadPerSec);
        table.put("DriveAppliedVolts",      this.driveAppliedVolts);
        table.put("DriveCurrentAmps",       this.driveCurrentAmps);
        table.put("TurnAbsolutePosition",   this.turnAbsolutePosition);
        table.put("TurnPosition",           this.turnPosition);
        table.put("TurnVelocityRadPerSec",  this.turnVelocityRadPerSec);
        table.put("TurnCurrentAmps",        this.turnCurrentAmps);
        table.put("TurnAppliedVolts",       this.turnAppliedVolts);
    }

    /**
     * Updates data based on a LogTable.
     * @param table The {@link LogTable} which is provided.
     */
    @Override
    public void fromLog(LogTable table) {
        this.drivePositionRad       = table.get("DrivePositionRad", this.drivePositionRad);
        this.driveVelocityRadPerSec = table.get("DriveVelocityRadPerSec", this.driveVelocityRadPerSec);
        this.driveAppliedVolts      = table.get("DriveAppliedVolts", this.driveAppliedVolts);
        this.driveCurrentAmps       = table.get("DriveCurrentAmps", this.driveCurrentAmps);
        this.turnAbsolutePosition   = table.get("TurnAbsolutePosition", this.turnAbsolutePosition);
        this.turnPosition           = table.get("TurnPosition", this.turnPosition);
        this.turnVelocityRadPerSec  = table.get("TurnVelocityRadPerSec", this.turnVelocityRadPerSec);
        this.turnCurrentAmps        = table.get("TurnCurrentAmps", this.turnCurrentAmps);
        this.turnAppliedVolts       = table.get("TurnAppliedVolts", this.turnAppliedVolts);
    }
}