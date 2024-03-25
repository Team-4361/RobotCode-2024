package frc.robot.util.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.DashTunablePID;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Robot;
import java.util.Optional;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.PHOTON_ENABLED;
import static frc.robot.Constants.Debug.PHOTON_TUNING_ENABLED;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

public class PhotonCameraModule extends PhotonCamera {

    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final String cameraName;
    private final PIDController driveController;
    private final PIDController turnController;
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private final DashTunableNumber speedTune;
    private final PhotonPoseEstimator poseEstimator;
    private final Transform3d cameraTransform;

    private Pose2d trackedPose = new Pose2d();
    private long lastFoundMillis = System.currentTimeMillis();
    private int aprilTagID = 0;
    private double maxDriveSpeed = PHOTON_DRIVE_MAX_SPEED;

    public double getMaxDriveSpeed() { return maxDriveSpeed; }
    public void setMaxDriveSpeed(double speed) { this.maxDriveSpeed = speed; }

    public PIDController getDriveController() { return this.driveController; }
    public PIDController getTurnController() { return this.turnController; }

    public PhotonCameraModule(String name, Transform3d transform) {
        super(name);
        this.cameraName = name;
        this.cameraTransform = transform;
        this.driveController = GlobalUtils.generateController(PHOTON_DRIVE_PID);
        this.turnController = GlobalUtils.generateController(PHOTON_TURN_PID);
        this.poseEstimator = new PhotonPoseEstimator(
                FIELD_LAYOUT,
                MULTI_TAG_PNP_ON_COPROCESSOR,
                cameraTransform
        );

        if (PHOTON_TUNING_ENABLED) {
            this.driveTune = new DashTunablePID("Photon: Drive PID", PHOTON_DRIVE_PID);
            this.turnTune = new DashTunablePID("Photon: Turn PID", PHOTON_TURN_PID);
            this.speedTune = new DashTunableNumber("Photon: Max Speed", PHOTON_DRIVE_MAX_SPEED);

            speedTune.addConsumer(this::setMaxDriveSpeed);
            driveTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
            turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);
        } else {
            driveTune = null;
            turnTune = null;
            speedTune = null;
        }
    }

    public Transform3d getCameraTransform() { return this.cameraTransform; }
    public Optional<Pose2d> getTrackedPose() { return Optional.ofNullable(trackedPose); }

    public void update() {
        if (!PHOTON_ENABLED || RobotBase.isSimulation())
            return;

        if (driveTune != null) driveTune.update();
        if (turnTune  != null) turnTune.update();
        if (speedTune != null) speedTune.update();

        PhotonPipelineResult result = getLatestResult();
        poseEstimator.update(result);

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