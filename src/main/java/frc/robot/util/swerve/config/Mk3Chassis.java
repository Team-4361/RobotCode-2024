package frc.robot.util.swerve.config;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.SwerveModule;

public class Mk3Chassis implements ChassisSettings {
    /**
     * @return The {@link ModuleSettings} associated with the front-left {@link SwerveModule}.
     */
    @Override
    public ModuleSettings getFLModule() {
        return new ModuleSettings(
                1,
                2,
                0,
                Rotation2d.fromRotations(6.254)
        );
    }

    /**
     * @return The {@link ModuleSettings} associated with the front-right {@link SwerveModule}.
     */
    @Override
    public ModuleSettings getFRModule() {
        return new ModuleSettings(
                3,
                4,
                1,
                Rotation2d.fromRotations(3.854)
        );
    }

    /**
     * @return The {@link ModuleSettings} associated with the back-left {@link SwerveModule}.
     */
    @Override
    public ModuleSettings getBLModule() {
        return new ModuleSettings(
                5,
                6,
                2,
                Rotation2d.fromRotations(2.867)
        );
    }

    /**
     * @return The {@link ModuleSettings} associated with the front-right {@link SwerveModule}.
     */
    @Override
    public ModuleSettings getBRModule() {
        return new ModuleSettings(
                7,
                8,
                3,
                Rotation2d.fromRotations(3.342)
        );
    }

    /** @return The Robot side length in meters. */
    @Override public double getSideLength() { return 0.762; }

    /** @return The Robot wheel radius in meters. */
    @Override public double getWheelRadius() { return 0.0508; }

    /** @return The GearRatio used for driving. */
    @Override public GearRatio getDriveRatio() { return GearRatio.from(6.86, 1); }

    /** @return The {@link GearRatio} used for turning. */
    @Override public GearRatio getTurnRatio() { return GearRatio.from(12.8, 1); }

    /** @return The maximum attainable speed of the Robot in m/s. */
    @Override public double getMaxSpeed() { return 12.5; }

    /** @return The PIDConstants used for closed-loop control. */
    @Override
    public PIDConstantsAK getDrivePID() {
        return new PIDConstantsAK(
                2e-4, 0, 0,
                0.1, 0, 0,
                0, 0, 0
        );
    }

    /** @return The PIDConstants used for turning. */
    @Override
    public PIDConstantsAK getTurnPID() {
        return new PIDConstantsAK(
                2, 0, 0,
                10, 0, 0,
                0, 0, 0
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
    @Override public boolean usingMagEncoders() { return true; }
}