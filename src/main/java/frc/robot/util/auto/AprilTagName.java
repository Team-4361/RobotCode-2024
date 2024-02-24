package frc.robot.util.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

public enum AprilTagName {
    // FIXME: add more entries!
    RED_SHOOTER(10, "Shooter", Red);







    private final int id;
    private final String name;
    private final Alliance alliance;

    public int getID() { return this.id; }
    public Alliance getAlliance() { return this.alliance; };

    public static Optional<AprilTagName> fromID(int id) {
        for (AprilTagName tag : AprilTagName.values()) {
            if (tag.id == id) {
                return Optional.of(tag);
            }
        }
        return Optional.empty();
    }

    @Override
    public String toString() {
        return name + " - " + alliance.name();
    }

    AprilTagName(int id, String name, Alliance alliance) {
        this.id = id;
        this.name = name;
        this.alliance = alliance;
    }
}
