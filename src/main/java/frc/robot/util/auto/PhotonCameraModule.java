package frc.robot.util.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.BaseSubsystem;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.TunableNumber;
import frc.robot.util.pid.TunablePID;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Robot;

import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.PHOTON_ENABLED;
import static frc.robot.Constants.Debug.PHOTON_TUNING_ENABLED;

public class PhotonCameraModule extends BaseSubsystem {

    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    protected static final String DRIVE_PID_NAME = "DrivePID";
    protected static final String TURN_PID_NAME = "TurnPID";

    private final Supplier<PIDController> drivePID;
    private final Supplier<PIDController> turnPID;
    private final Transform3d cameraTransform;
    private final PhotonCamera camera;

    private Pose2d trackedPose = new Pose2d();
    private long lastFoundMillis = System.currentTimeMillis();
    private int aprilTagID = 0;

    public double getMaxDriveSpeed() { return getTargetSpeed(); }
    public void setMaxDriveSpeed(double speed) { setTargetSpeed(speed); }

    public PIDController getDriveController() { return drivePID.get(); }
    public PIDController getTurnController() { return turnPID.get(); }

    public PhotonCameraModule(String name, Transform3d transform) {
        super(
                name,
                PHOTON_DRIVE_MAX_SPEED,
                PHOTON_TUNING_ENABLED,
                new HashMap<>()
        );
        this.camera = new PhotonCamera(name);
        this.drivePID = registerPID(DRIVE_PID_NAME, PHOTON_DRIVE_PID);
        this.turnPID = registerPID(TURN_PID_NAME, PHOTON_TURN_PID);
        this.cameraTransform = transform;
    }

    public Transform3d getCameraTransform() { return this.cameraTransform; }
    public Optional<Pose2d> getTrackedPose() { return Optional.ofNullable(trackedPose); }

    public void update() {
        if (!PHOTON_ENABLED || RobotBase.isSimulation())
            return;

        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            aprilTagID = target.getFiducialId();
            lastFoundMillis = System.currentTimeMillis();
            trackedPose = poseEstimator
                    .getReferencePose()
                    .toPose2d();

            // Update the pose estimator with the information.
            Robot.swerve.addVisionMeasurement(trackedPose, Timer.getFPGATimestamp());

            Optional<AprilTagID> name = AprilTagID.fromID(aprilTagID);
            if (name.isPresent()) {
                SmartDashboard.putString("Found Tag", name.get().toString());
            } else {
                SmartDashboard.putString("Found Tag", "NONE");
            }
        } else {
            if (System.currentTimeMillis() >= lastFoundMillis+500) {
                trackedPose = null;
                SmartDashboard.putString("Found Tag", "NONE");
            }
        }
    }

    public String getCameraName() { return this.cameraName; }
    public boolean isTargetFound() { return trackedPose != null; }
    public long getLastFoundMillis() { return this.lastFoundMillis; }
    public int getAprilTagID() { return this.aprilTagID; }
}