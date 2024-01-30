package frc.robot.util.swerve;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.io.IOManager;
import frc.robot.util.pid.DashTunablePID;
import frc.robot.util.swerve.config.SwerveModuleIO;
import frc.robot.util.swerve.config.SwerveModuleIO.SwerveModuleIOInputs;
import frc.robot.util.swerve.config.SwerveModuleIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

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
public class SwerveModule {
    private Rotation2d turnRelativeOffset = null;
    private Rotation2d angleSetpoint = null;
    private Double speedSetpoint = null;

    private final PIDConstants drivePIDConfig;
    private final PIDConstants turnPIDConfig;
    private final SimpleMotorFeedforward driveFF;
    private final PIDController driveController;
    private final PIDController turnController;
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private final String name;
    private final SwerveModuleIO io;
    private final SwerveModuleIOInputsAutoLogged inputs = new SwerveModuleIOInputsAutoLogged();

    //public static final DashTunablePID driveTune = new DashTunablePID("Drive PID", DRIVE_PID_CONFIG);
    //public static final DashTunablePID steerTune = new DashTunablePID("Steer PID", TURN_PID_CONFIG);
    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name            The name of the swerve module.
     * @param io              The {@link SwerveModuleIO} adapter used for direct communication.
     * @param drivePIDConfig  The {@link PIDConstants} to use for closed-loop driving.
     * @param turnPIDConfig   The {@link PIDConstants} to use for PWM turning.
     */
    public SwerveModule(String name, SwerveModuleIO io, PIDConstants drivePIDConfig, PIDConstants turnPIDConfig) {
        this.driveController = new PIDController(drivePIDConfig.kP, drivePIDConfig.kI, drivePIDConfig.kD);
        this.turnController  = new PIDController(turnPIDConfig.kP, turnPIDConfig.kI, turnPIDConfig.kD);
        this.driveFF         = new SimpleMotorFeedforward(0.1, 0.13, 0);



        this.drivePIDConfig = drivePIDConfig;
        this.turnPIDConfig = turnPIDConfig;
        this.name = name;
        this.io = io;

        // Shorten the travel as much as possible for efficency reasons.
        turnController.enableContinuousInput(-Math.PI, Math.PI);

        if (SWERVE_TUNING_ENABLED) {
            // PID tuning is enabled.
            driveTune = new DashTunablePID(name + ": Drive PID", drivePIDConfig);
            turnTune = new DashTunablePID(name + ": Turn PID", turnPIDConfig);

            driveTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
            turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);

            IOManager.addPeriodicIfExists(STRING_DASHBOARD_NAME, () -> {
                driveTune.update();
                turnTune.update();
            });
        } else {
            driveTune = null;
            turnTune = null;
        }
    }

    public void update() {
        io.updateInputs(inputs);
        Logger.processInputs("Drive/Module" + name, inputs);
        if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0) {
            turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);
        }

        // Run closed loop turn control
        if (angleSetpoint != null) {
            io.setTurnVoltage(
                    turnController.calculate(getAngle().getDegrees(), angleSetpoint.getDegrees()));

            // Run closed loop drive control
            // Only allowed if closed loop turn control is running
            if (speedSetpoint != null) {
                // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
                // towards the setpoint, its velocity should increase. This is achieved by
                // taking the component of the velocity in the direction of the setpoint.
                double adjustSpeedSetpoint = speedSetpoint * Math.cos(turnController.getPositionError());

                // Run drive controller
                double velocityRadPerSec = adjustSpeedSetpoint / CHASSIS_MODE.getWheelRadius();
                io.setDriveVoltage(
                        driveFF.calculate(velocityRadPerSec)
                            + driveController.calculate(inputs.driveVelocityRadPerSec, velocityRadPerSec)
                );
            }
        }
    }

    public Rotation2d getAngle() {
        if (turnRelativeOffset == null) {
            return new Rotation2d();
        } else {
            //return inputs.turnPosition.plus(turnRelativeOffset);
            return inputs.turnAbsolutePosition;
        }
    }

    /** Returns the current drive velocity of the module in meters per second. */
    public double getVelocityMetersPerSec() {
        return inputs.driveVelocityRadPerSec * CHASSIS_MODE.getWheelRadius();
    }


    /** Returns the drive velocity in radians/sec. */
    public double getCharacterizationVelocity() {
        return inputs.driveVelocityRadPerSec;
    }

    /** @return The {@link PIDConstants} to use for closed-loop driving. */
    public PIDConstants getDrivePIDConfig() { return this.drivePIDConfig; }

    /** @return The {@link PIDConstants} to use for PWM turning. */
    public PIDConstants getTurnPIDConfig() { return this.turnPIDConfig; }

    /**
     * Sets the state of the {@link SwerveModule}.
     *
     * @param state The {@link SwerveModuleState} to use.
     */
    public SwerveModuleState setState(SwerveModuleState state, boolean isClosedLoop) {
        // Optimize state based on current angle
        // Controllers run in "periodic" when the setpoint is not null
        var optimizedState = SwerveModuleState.optimize(state, getAngle());

        // Update setpoints, controllers run in "periodic"

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

        io.setDriveVoltage(volts);
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
        return inputs.drivePositionRad * CHASSIS_MODE.getWheelRadius();
    }

    /** Resets the relative drive encoder reading on the {@link SwerveModule}. */
    public void reset() { io.reset(); }
}