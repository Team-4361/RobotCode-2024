package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.util.io.IOManager;
import org.photonvision.PhotonCamera;

import static frc.robot.Constants.LooperConfig.STRING_ODOMETRY_NAME;

public class PhotonCameraModule extends PhotonCamera {
    public static final int CAMERA_BUFFER_MILLIS = 500;

    private final double cameraHeight;
    private final double cameraPitch;
    private final String cameraName;
    private Transform3d targetTransform;
    private Pose2d trackedPose;
    private boolean targetFound;

    public PhotonCameraModule(String name, double height, double pitch) {
        super(name);
        this.cameraName = name;
        this.cameraHeight = height;
        this.cameraPitch = pitch;

        IOManager.addPeriodicIfExists(STRING_ODOMETRY_NAME, () -> {

        });
    }

    public double getCameraHeight() { return this.cameraHeight; }
    public double getCameraPitch() { return this.cameraPitch; }
    public Pose2d getTrackedPose() { return this.trackedPose; }
}
