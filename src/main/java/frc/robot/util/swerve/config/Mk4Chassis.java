package frc.robot.util.swerve.config;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.SwerveModuleBase;

import static frc.robot.Constants.Mk4SDSRatio.L2;

public class Mk4Chassis implements ChassisSettings {
    /**
     * @return The {@link ModuleSettings} associated with the front-left {@link SwerveModuleBase}.
     */
    @Override
    public ModuleSettings getFLModule() {
        return new ModuleSettings(
                2,
                1,
                30,
                0
        );
    }

    /**
     * @return The {@link ModuleSettings} associated with the front-right {@link SwerveModuleBase}.
     */
    @Override
    public ModuleSettings getFRModule() {
        return new ModuleSettings(
                4,
                3,
                31,
                0
        );
    }

    /**
     * @return The {@link ModuleSettings} associated with the back-left {@link SwerveModuleBase}.
     */
    @Override
    public ModuleSettings getBLModule() {
        return new ModuleSettings(
                6,
                5,
                32,
                0
        );
    }

    /**
     * @return The {@link ModuleSettings} associated with the front-right {@link SwerveModuleBase}.
     */
    @Override
    public ModuleSettings getBRModule() {
        return new ModuleSettings(
                8,
                7,
                33,
                0
        );
    }

    /** @return The Robot side length in meters. */
    @Override public double getSideLength() { return Units.inchesToMeters(30); }

    /** @return The Robot wheel radius in meters. */
    @Override public double getWheelRadius() { return 0.0508; }

    /** @return The GearRatio used for driving. */
    @Override public GearRatio getDriveRatio() { return GearRatio.from(L2.getRatio(), 1); }

    /** @return The {@link GearRatio} used for turning. */
    @Override public GearRatio getTurnRatio() { return GearRatio.from(12.8, 1); }

    /** @return The maximum attainable speed of the Robot in m/s. */
    @Override public double getMaxSpeed() { return 12.5; }

    /** @return The PIDConstants used for closed-loop control. */
    @Override
    public PIDConstantsAK getDrivePID() {
        return new PIDConstantsAK(
                2e-4, 0, 0,
                0.1, 0, 0
        );
    }

    /** @return The PIDConstants used for turning. */
    @Override
    public PIDConstantsAK getTurnPID() {
        return new PIDConstantsAK(
                2, 0, 0,
                10, 0, 0
        );
    }

    /** @return The PIDConstants used for PathPlannerAuto closed-loop control. */
    @Override
    public PIDConstantsAK getAutoDrivePID() {
        // TODO: tuning is required!
        return new PIDConstantsAK(5, 0, 0);
    }

    /** @return The PIDConstants used for PathPlannerAuto turning. */
    @Override
    public PIDConstantsAK getAutoTurnPID() {
        // TODO: tuning is required!
        return new PIDConstantsAK(1,0,0);
    }

    /** @return The PIDConstants used for PhotonCamera closed-loop control. */
    @Override
    public PIDConstantsAK getPhotonDrivePID() {
        // TODO: tuning is required!
        return new PIDConstantsAK(0.5, 0, 0);
    }

    /** @return The PIDConstants used for PhotonCamera turning. */
    @Override
    public PIDConstantsAK getPhotonTurnPID() {
        // TODO: tuning is required!
        return new PIDConstantsAK(0.1, 0, 0);
    }

    /** @return If the legacy CTRE magnetic encoders are being used. */
    @Override public boolean usingMagEncoders() { return false; }

    /** @return The {@link SimpleMotorFeedforward} used for the SwerveModule. */
    @Override
    public SimpleMotorFeedforward getFeedForward() {
        return new SimpleMotorFeedforward(0.1, 0.13, 0);
    }
}