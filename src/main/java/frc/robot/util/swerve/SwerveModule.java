package frc.robot.util.swerve;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.motor.FRCSparkMax;

import static com.revrobotics.CANSparkBase.ControlType.kVelocity;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;
import static frc.robot.Constants.Chassis.*;

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
    private final FRCSparkMax driveMotor;
    private final FRCSparkMax turnMotor;
    private final RelativeEncoder driveEncoder;
    private final DutyCycleEncoder rotationPWMEncoder;
    private final double offsetRads;
    private final PIDConstants drivePIDConfig;
    private final PIDConstants turnPIDConfig;
    private final PIDController turnController;
    private final SparkPIDController driveController;

    /**
     * Creates a new {@link SwerveModule} instance using the specified parameters. The {@link CANSparkMax}
     * motor instance will be <b>created and reserved.</b>
     *
     * @param driveMotorId       The Motor ID used for driving the wheel.
     * @param turnMotorId        The Motor ID used for turning the wheel.
     * @param digitalEncoderPort The {@link DigitalInput} ID used for the Encoder.
     * @param offsetRads         The offset to use for driving the wheel in <b>radians</b>.
     * @param drivePIDConfig     The {@link PIDConstants} to use for closed-loop driving.
     * @param turnPIDConfig      The {@link PIDConstants} to use for PWM turning.
     */
    public SwerveModule(int driveMotorId, int turnMotorId, int digitalEncoderPort,
                        double offsetRads, PIDConstants drivePIDConfig, PIDConstants turnPIDConfig) {

        this.driveMotor = new FRCSparkMax(driveMotorId, kBrushless, DCMotor.getNEO(1));
        this.turnMotor = new FRCSparkMax(turnMotorId, kBrushless, DCMotor.getNEO(1));

        this.rotationPWMEncoder = new DutyCycleEncoder(digitalEncoderPort);
        this.turnController = new PIDController(turnPIDConfig.kP, turnPIDConfig.kI, turnPIDConfig.kD, turnPIDConfig.kP);

        this.offsetRads = offsetRads;
        this.drivePIDConfig = drivePIDConfig;
        this.turnPIDConfig = turnPIDConfig;

        this.driveEncoder = driveMotor.getEncoder();
        this.driveController = driveMotor.getPIDController();

        // Set the PID config for driving.
        driveController.setP(drivePIDConfig.kP);
        driveController.setI(drivePIDConfig.kI);
        driveController.setD(drivePIDConfig.kD);
        driveController.setP(drivePIDConfig.kP);
    }

    /** @return The current {@link SwerveModule} velocity in meters per second. */
    public double getVelocity() {
        return DRIVE_GEAR_RATIO.getFollowerRotation(
                driveEncoder.getVelocity() / 60) * SWERVE_WHEEL_CIRCUMFERENCE;
    }

    /** @return The {@link PIDConstants} to use for closed-loop driving. */
    public PIDConstants getDrivePIDConfig() { return this.drivePIDConfig; }

    /** @return The {@link PIDConstants} to use for PWM turning. */
    public PIDConstants getTurnPIDConfig() { return this.turnPIDConfig; }

    /** @return The current {@link SwerveModule} Turn Angle in radians. */
    public double getTurnAngle() { return offsetRads + (rotationPWMEncoder.get() * 2 * Math.PI); }

    public void setState(SwerveModuleState state, boolean isClosedLoop) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromRadians(getTurnAngle()));

        if (isClosedLoop) {
            // Set the desired RPM to achieve the meters per second.
            double desiredRPM = 60 * DRIVE_GEAR_RATIO.getLeadRotation(
                    (state.speedMetersPerSecond * 60) / SWERVE_WHEEL_CIRCUMFERENCE);
            double maxRPM = Units.radiansPerSecondToRotationsPerMinute(driveMotor.getModel().freeSpeedRadPerSec);

            driveController.setReference(MathUtil.clamp(desiredRPM, -maxRPM, maxRPM), kVelocity);
        } else {
            driveMotor.set(MathUtil.clamp(state.speedMetersPerSecond * MAX_SPEED_MPS, -1, 1));
        }

        turnMotor.set(MathUtil.clamp(turnController.calculate(getTurnAngle(), state.angle.getRadians()), -1, 1));
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
    public SwerveModulePosition getPosition()  {
        return new SwerveModulePosition(
                getDistance(),
                Rotation2d.fromRadians(getTurnAngle())
        );
    }

    public void updateDashboard(String prefix) {
        String driveVelocity = prefix + ": motor rpm";
        String drivePower = prefix + ": pow";
        String turnPower = prefix + ": turn pow";
        String turnPosition = prefix + ": turn rad";
        String drivePosition = prefix + ": distance";

        SmartDashboard.putNumber(driveVelocity, driveEncoder.getVelocity());
        SmartDashboard.putNumber(turnPower, turnMotor.get());
        SmartDashboard.putNumber(turnPosition, getTurnAngle());
        SmartDashboard.putNumber(drivePower, driveMotor.get());
        SmartDashboard.putNumber(drivePosition, getDistance());
    }

    /** 
     * @return The total amount of meters the individual {@link SwerveModule} has traveled.
     */
    public double getDistance() {
        // The formula for calculating meters from total rotation is:
        // (Total Rotations * 2PI * Wheel Radius)
        return (DRIVE_GEAR_RATIO.getFollowerRotation(driveEncoder.getPosition()) * (2 * Math.PI) * SWERVE_WHEEL_RADIUS);
    }

    public void reset() { driveEncoder.setPosition(0); }
}