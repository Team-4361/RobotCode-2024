package frc.robot.util.auto;

import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;

import static edu.wpi.first.wpilibj.DriverStation.Alliance.Blue;
import static edu.wpi.first.wpilibj.DriverStation.Alliance.Red;

public enum AprilTagName {

    RED_SPEAKER_MID(4, "Speaker-Mid", Red),
    RED_SPEAKER_LEFT(3, "Speaker-Left", Red),
    RED_SOURCE_LEFT(10, "Source-Left", Red),
    RED_SOURCE_RIGHT(9, "Source-Right", Red),
    RED_AMP(5, "Amp", Red),
    RED_STAGE_LEFT(11, "Stage-Left", Red),
    RED_STAGE_RIGHT(12, "Stage-Right", Red),
    RED_STAGE_MID(13, "Stage-Mid", Red),
    BLUE_SOURCE_RIGHT(1, "Source-Right", Blue),
    BLUE_SOURCE_LEFT(2, "Source-Left", Blue),
    BLUE_SPEAKER_MID(7, "Speaker-Mid", Blue),
    BLUE_SPEAKER_RIGHT(8, "Speaker-Right", Blue),
    BLUE_AMP(6, "Amp", Blue),
    BLUE_STAGE_MID(14, "Stage-Mid", Blue),
    BLUE_STAGE_LEFT(15, "Stage-Left", Blue),
    BLUE_STAGE_RIGHT(16, "Stage-Right", Blue);









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
