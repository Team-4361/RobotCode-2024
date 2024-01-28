package frc.robot.util.swerve.config;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PIDConstants;
import frc.robot.Robot;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PIDConstantsAK;
import org.photonvision.PhotonCamera;

/**
 * This {@link ChassisSettings} interface is designed to store values regarding Driving the robot. This includes any offsets
 * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
 * need to be flipped due to the Field Oriented driving system.
 */
public interface ChassisSettings {
    /** @return The front-left offset. */
    double getFLOffsetRad();

    /** @return The front-right offset. */
    double getFROffset();

    /** @return The back-left offset. */
    double getBLOffset();

    /** @return The back-right offset. */
    double getBROffset();

    /** @return The {@link Robot} side length in <b>meters.</b> */
    double getSideLength();

    /** @return The front-left drive ID. */
    int getFLDriveID();

    /** @return The front-right drive ID. */
    int getFRDriveID();

    /** @return The back-left drive ID. */
    int getBLDriveID();

    /** @return The back-right drive ID. */
    int getBRDriveID();

    /** @return The front-left turn ID. */
    int getFLTurnID();

    /** @return The front-right turn ID. */
    int getFRTurnID();

    /** @return The back-left turn ID. */
    int getBLTurnID();

    /** @return The back-right turn ID. */
    int getBRTurnID();

    /** @return The encoder ID of the front-left module. */
    int getFLEncoderID();

    /** @return The encoder ID of the front-right module. */
    int getFREncoderID();

    /** @return The encoder ID of the back-left module. */
    int getBLEncoderID();

    /** @return The encoder ID of the back-right module. */
    int getBREncoderID();

    /** @return The {@link Robot} wheel radius in <b>meters.</b> */
    double getWheelRadius();

    /** @return The {@link GearRatio} used for driving. */
    GearRatio getDriveRatio();

    /** @return The {@link GearRatio} used for turning. */
    GearRatio getTurnRatio();

    /** @return The maximum attainable speed of the {@link Robot} in m/s. */
    double getMaxSpeed();

    /** @return The {@link PIDConstants} used for closed-loop control. */
    PIDConstantsAK getDrivePID();

    /** @return The {@link PIDConstants} used for turning. */
    PIDConstantsAK getTurnPID();

    /** @return The {@link PIDConstants} used for {@link PathPlannerAuto} closed-loop control. */
    PIDConstantsAK getAutoDrivePID();

    /** @return The {@link PIDConstants} used for {@link PathPlannerAuto} turning. */
    PIDConstantsAK getAutoTurnPID();

    /** @return The {@link PIDConstants} used for {@link PhotonCamera} closed-loop control. */
    PIDConstantsAK getPhotonDrivePID();

    /** @return The {@link PIDConstants} used for {@link PhotonCamera} turning. */
    PIDConstantsAK getPhotonTurnPID();

    /** @return If the legacy CTRE magnetic encoders are being used. */
    boolean usingMagEncoders();
}

