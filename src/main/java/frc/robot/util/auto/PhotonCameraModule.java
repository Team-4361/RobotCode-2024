package frc.robot.util.auto;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.DashTunablePID;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.Robot;
import java.util.Optional;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.PHOTON_ENABLED;
import static frc.robot.Constants.Debug.PHOTON_TUNING_ENABLED;

public class PhotonCameraModule extends PhotonCamera {

    public static final AprilTagFieldLayout FIELD_LAYOUT =
            AprilTagFields.k2024Crescendo.loadAprilTagLayoutField();

    private final String cameraName;
    private final PIDController driveController;
    private final PIDController turnController;
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private final DashTunableNumber speedTune;
    private final Transform3d cameraTransform;
    private Transform3d targetTransform;

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

        if (PHOTON_TUNING_ENABLED) {
            this.driveTune = new DashTunablePID(name + ": Drive PID", PHOTON_DRIVE_PID);
            this.turnTune = new DashTunablePID(name + ": Turn PID", PHOTON_TURN_PID);
            this.speedTune = new DashTunableNumber(name + ": Max Speed", PHOTON_DRIVE_MAX_SPEED);

            speedTune.addConsumer(this::setMaxDriveSpeed);
            driveTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
            turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);
        } else {
            driveTune = null;
            turnTune = null;
            speedTune = null;
        }
    }

    public Transform3d getCameraTransform() { return cameraTransform; }
    public Optional<Transform3d> getTargetTransform() { return Optional.ofNullable(targetTransform); }

    public void update() {
        if (!PHOTON_ENABLED || RobotBase.isSimulation())
            return;

        if (driveTune != null) driveTune.update();
        if (turnTune  != null) turnTune.update();
        if (speedTune != null) speedTune.update();

        PhotonPipelineResult result = getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            aprilTagID = target.getFiducialId();
            lastFoundMillis = System.currentTimeMillis();
            targetTransform = target.getBestCameraToTarget();
        } else {
            if (System.currentTimeMillis() >= lastFoundMillis + 500) {
                targetTransform = null;
            }
        }
        if (targetTransform == null)
            SmartDashboard.putString("Tracked Pose", "NONE");
        else
            SmartDashboard.putString("Tracked Pose", targetTransform.toString());
    }

    public String getCameraName() { return this.cameraName; }
    public boolean isTargetFound() { return targetTransform != null; }
    public long getLastFoundMillis() { return this.lastFoundMillis; }
    public int getAprilTagID() { return this.aprilTagID; }
}