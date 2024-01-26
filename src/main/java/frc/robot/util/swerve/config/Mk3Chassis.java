package frc.robot.util.swerve.config;

import com.pathplanner.lib.util.PIDConstants;
import frc.robot.util.math.GearRatio;

public class Mk3Chassis implements ChassisSettings {
    /** @return The front-left offset. */
    @Override public double getFLOffset() { return ((9.401)+0.045647)+(Math.PI/2) - (Math.PI / 2); }

    /** @return The front-right offset. */
    @Override public double getFROffset() { return  ((-2.38)+0)+(Math.PI/2) - (2 * Math.PI) + (Math.PI); }

    /** @return The back-left offset. */
    @Override public double getBLOffset() { return ((6.12)+0.339057)+(Math.PI/2) - (2 * Math.PI) - (Math.PI / 2); }

    /** @return The back-right offset. */
    @Override public double getBROffset() { return ((-3.345)+0.009)+(Math.PI/2) - (Math.PI / 2) - (2 * Math.PI); }

    /** @return The Robot side length in meters. */
    @Override public double getSideLength() { return 0.762; }

    /** @return The front-left drive ID. */
    @Override public int getFLDriveID() { return 2; }

    /** @return The front-right drive ID. */
    @Override public int getFRDriveID() { return 4; }

    /** @return The back-left drive ID. */
    @Override public int getBLDriveID() { return 6; }

    /** @return The back-right drive ID. */
    @Override public int getBRDriveID() { return 8; }

    /** @return The front-left turn ID. */
    @Override public int getFLTurnID() { return 1; }

    /** @return The front-right turn ID. */
    @Override public int getFRTurnID() { return 3; }

    /** @return The back-left turn ID. */
    @Override public int getBLTurnID() { return 5; }

    /** @return The back-right turn ID. */
    @Override public int getBRTurnID() { return 7; }

    /** @return The encoder ID of the front-left module. */
    @Override public int getFLEncoderID() { return 0; }

    /** @return The encoder ID of the front-right module. */
    @Override public int getFREncoderID() { return 1; }

    /** @return The encoder ID of the back-left module. */
    @Override public int getBLEncoderID() { return 2; }

    /** @return The encoder ID of the back-right module. */
    @Override public int getBREncoderID() { return 3; }

    /** @return The Robot wheel radius in meters. */
    @Override public double getWheelRadius() { return 0.0508; }

    /** @return The GearRatio used for driving. */
    @Override public GearRatio getDriveRatio() { return GearRatio.from(6.86, 1); }

    /** @return The maximum attainable speed of the Robot in m/s. */
    @Override public double getMaxSpeed() { return 12.5; }

    /** @return The PIDConstants used for closed-loop control. */
    @Override public PIDConstants getDrivePID() { return new PIDConstants(2e-4, 0, 0); }

    /** @return The PIDConstants used for turning. */
    @Override public PIDConstants getTurnPID() { return new PIDConstants(0.5, 0, 0); }

    /** @return The PIDConstants used for PathPlannerAuto closed-loop control. */
    @Override public PIDConstants getAutoDrivePID() { return new PIDConstants(5, 0, 0); }

    /** @return The PIDConstants used for PathPlannerAuto turning. */
    @Override public PIDConstants getAutoTurnPID() { return new PIDConstants(1,0,0); }

    /** @return The PIDConstants used for PhotonCamera closed-loop control. */
    @Override public PIDConstants getPhotonDrivePID() { return new PIDConstants(0.5, 0, 0); }

    /** @return The PIDConstants used for PhotonCamera turning. */
    @Override public PIDConstants getPhotonTurnPID() { return new PIDConstants(0.1, 0, 0); }

    /** @return If the legacy CTRE magnetic encoders are being used. */
    @Override public boolean usingMagEncoders() { return true; }
}