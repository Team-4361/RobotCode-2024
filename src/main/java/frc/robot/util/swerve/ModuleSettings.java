package frc.robot.util.swerve;

/**
 * This {@link ModuleSettings} interface is designed to store values regarding Driving the robot. This includes any offsets
 * for Absolute Driving, dead-zones, and ports regarding the motors. Note that these motors usually <b>do not</b>
 * need to be flipped due to the Field Oriented driving system.
 */
public class ModuleSettings {
    private final double absOffset;
    private final int driveID;
    private final int turnID;
    private final int encoderID;

    public double getOffsetRotations() { return absOffset; }
    public int getDriveID() { return driveID; }
    public int getTurnID() { return turnID; }
    public int getEncoderID() { return encoderID; }

    /**
     * Constructs a new {@link ModuleSettings} instance.
     * @param driveID The drive ID 
     * @param turnID The turn ID
     * @param encoderID The encoder ID
     * @param offsetRot The offset in rotationss.
     */
    public ModuleSettings(int driveID, int turnID, int encoderID, double offsetRot) {
        this.driveID = driveID;
        this.turnID = turnID;
        this.encoderID = encoderID;
        this.absOffset = offsetRot;
    }
}

