package frc.robot.util.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.io.IOManager;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.MotorModel;
import frc.robot.util.pid.DashTunablePID;
import frc.robot.util.swerve.config.SwerveModuleIO;
import frc.robot.util.swerve.config.SwerveModuleIO.SwerveModuleIOInputs;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Control.*;
import static frc.robot.Constants.LooperConfig.STRING_PERIODIC_NAME;

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
    private final double offsetRads;
    private final PIDConstants drivePIDConfig;
    private final PIDConstants turnPIDConfig;
    private final PIDController turnController;
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private final String name;
    private final SwerveModuleIO io;
    private SwerveModuleIOInputs inputs;

    //public static final DashTunablePID driveTune = new DashTunablePID("Drive PID", DRIVE_PID_CONFIG);
    //public static final DashTunablePID steerTune = new DashTunablePID("Steer PID", TURN_PID_CONFIG);
    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param name            The name of the swerve module.
     * @param io              The {@link SwerveModuleIO} adapter used for direct communication.
     * @param offsetRads      The offset to use for driving the wheel in <b>radians</b>.
     * @param drivePIDConfig  The {@link PIDConstants} to use for closed-loop driving.
     * @param turnPIDConfig   The {@link PIDConstants} to use for PWM turning.
     */
    public SwerveModule(String name, SwerveModuleIO io,
                        double offsetRads, PIDConstants drivePIDConfig, PIDConstants turnPIDConfig) {
        this.turnController = new PIDController(turnPIDConfig.kP, turnPIDConfig.kI, turnPIDConfig.kD, turnPIDConfig.kP);

        this.offsetRads = offsetRads;
        this.drivePIDConfig = drivePIDConfig;
        this.turnPIDConfig = turnPIDConfig;
        this.name = name;
        this.io = io;

        // Shorten the travel as much as possible for efficency reasons.
        turnController.enableContinuousInput(0, 90);

        // Set the PID config for driving.
        io.setPID(drivePIDConfig);

        if (SWERVE_TUNING_ENABLED) {
            // PID tuning is enabled.
            driveTune = new DashTunablePID(name + ": Drive PID", drivePIDConfig);
            turnTune = new DashTunablePID(name + ": Turn PID", turnPIDConfig);
            //driveTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
            turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);
        } else {
            driveTune = null;
            turnTune = null;
        }
    }

    public void update() { io.updateInputs(inputs); }

    /**
     * @return The current {@link SwerveModule} velocity in meters per second.
     */
    public double getVelocity() {
        return CHASSIS_MODE.getDriveRatio().getFollowerRotations(
                inputs.driveRPM / 60) * (2 * Math.PI * CHASSIS_MODE.getWheelRadius()) ;
    }

    /** @return The {@link PIDConstants} to use for closed-loop driving. */
    public PIDConstants getDrivePIDConfig() { return this.drivePIDConfig; }

    /** @return The {@link PIDConstants} to use for PWM turning. */
    public PIDConstants getTurnPIDConfig() { return this.turnPIDConfig; }

    /** @return The current {@link SwerveModule} Turn Angle in radians. */
    public double getTurnAngle() { return offsetRads + (inputs.turnRotations * 2 * Math.PI); }

    /**
     * Sets the state of the {@link SwerveModule}.
     *
     * @param state The {@link SwerveModuleState} to use.
     * @param isClosedLoop If closed-loop driving control should be used.
     */
    public void setState(SwerveModuleState state, boolean isClosedLoop) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getTurnAngle()));

        if (isClosedLoop) {
            // Set the desired RPM to achieve the meters per second.
            double desiredRPM = io.getFreeSpeedRPM() * (state.speedMetersPerSecond / CHASSIS_MODE.getMaxSpeed());
            io.driveClosedLoop(desiredRPM);
        } else {
            io.driveOpenLoop(
                    MathUtil.clamp(state.speedMetersPerSecond / CHASSIS_MODE.getMaxSpeed(), -1, 1)
            );
        }
        io.setTurnPower(MathUtil.clamp(turnController.calculate(getTurnAngle(), state.angle.getRadians()), -1, 1));
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
                getVelocity(),
                Rotation2d.fromRadians(getTurnAngle())
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
                Rotation2d.fromRadians(getTurnAngle())
        );
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
        return (CHASSIS_MODE
                .getDriveRatio()
                .getFollowerRotations(inputs.driveRotations) * (2 * Math.PI * CHASSIS_MODE.getWheelRadius()));
    }

    /** Resets the relative drive encoder reading on the {@link SwerveModule}. */
    public void reset() { io.reset(); }
}