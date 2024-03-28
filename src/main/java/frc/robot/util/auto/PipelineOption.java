package frc.robot.util.auto;

import com.pathplanner.lib.util.PIDConstants;

public record PipelineOption(
        String name,
        int cameraIndex,
        boolean isAprilTag,
        double targetHeight,
        PIDConstants drivePID,
        PIDConstants turnPID
) {}
