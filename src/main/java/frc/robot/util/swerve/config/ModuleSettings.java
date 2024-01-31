package frc.robot.util.swerve.config;

import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Robot;
import frc.robot.util.math.GearRatio;
import frc.robot.util.pid.PIDConstantsAK;
import org.photonvision.PhotonCamera;

/**
 * This {@link ModuleSettings} interface is designed to store values regarding Driving the robot. This includes any offsets
 * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
 * need to be flipped due to the Field Oriented driving system.
 */
public class ModuleSettings {
    private final Rotation2d absOffset;
    private final int driveID;
    private final int turnID;
    private final int encoderID;

    public Rotation2d getOffset() { return absOffset; }
    public int getDriveID() { return driveID; }
    public int getTurnID() { return turnID; }
    public int getEncoderID() { return encoderID; }

    public ModuleSettings(int driveID, int turnID, int encoderID, Rotation2d offset) {
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
        this.absOffset = offset;
    }
}

