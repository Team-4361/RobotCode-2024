package frc.robot.util.swerve.config;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PIDConstantsAK;
import frc.robot.util.swerve.SwerveModuleBase;

public interface ChassisSettings {
    /** @return The {@link ModuleSettings} associated with the front-left {@link SwerveModuleBase}. */
    ModuleSettings getFLModule();

    /** @return The {@link ModuleSettings} associated with the front-right {@link SwerveModuleBase}. */
    ModuleSettings getFRModule();

    /** @return The {@link ModuleSettings} associated with the back-left {@link SwerveModuleBase}. */
    ModuleSettings getBLModule();

    /** @return The {@link ModuleSettings} associated with the front-right {@link SwerveModuleBase}. */
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

    /** @return The {@link SimpleMotorFeedforward} used for the SwerveModule. */
    SimpleMotorFeedforward getFeedForward();

    /** @return If the legacy CTRE magnetic encoders are being used. */
    boolean usingMagEncoders();
}
