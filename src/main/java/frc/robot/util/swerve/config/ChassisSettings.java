package frc.robot.util.swerve.config;

import com.pathplanner.lib.util.PIDConstants;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.SwerveModule;

public interface ChassisSettings {
    /** @return The {@link ModuleSettings} associated with the front-left {@link SwerveModule}. */
    ModuleSettings getFLModule();

    /** @return The {@link ModuleSettings} associated with the front-right {@link SwerveModule}. */
    ModuleSettings getFRModule();

    /** @return The {@link ModuleSettings} associated with the back-left {@link SwerveModule}. */
    ModuleSettings getBLModule();

    /** @return The {@link ModuleSettings} associated with the front-right {@link SwerveModule}. */
    ModuleSettings getBRModule();

    /** @return The Robot side length in meters. */
    double getSideLength();

    /** @return The maximum attainable speed of the Robot in m/s. */

    double getMaxSpeed();

    /** @return The Robot wheel radius in meters. */
    double getWheelRadius();

    /** @return The {@link GearRatio} used for driving. */
    GearRatio getDriveRatio();

    /** @return The {@link GearRatio} used for turning. */
    GearRatio getTurnRatio();

    /** @return The PIDConstants used for closed-loop control. */
    PIDConstantsAK getDrivePID();

    /** @return The PIDConstants used for turning. */
    PIDConstantsAK getTurnPID();

    /** @return The PIDConstants used for PathPlannerAuto closed-loop control. */
    PIDConstantsAK getAutoDrivePID();

    /** @return The PIDConstants used for PathPlannerAuto turning. */
    PIDConstantsAK getAutoTurnPID();

    /** @return The PIDConstants used for PhotonCamera closed-loop control. */
    PIDConstantsAK getPhotonDrivePID();

    /** @return The PIDConstants used for PhotonCamera turning. */
    PIDConstantsAK getPhotonTurnPID();

    /** @return If the legacy CTRE magnetic encoders are being used. */
    boolean usingMagEncoders();
}
