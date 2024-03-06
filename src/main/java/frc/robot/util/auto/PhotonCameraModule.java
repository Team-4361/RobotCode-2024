package frc.robot.util.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.DashTunablePID;

import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.PHOTON_ENABLED;
import static frc.robot.Constants.Debug.PHOTON_TUNING_ENABLED;

public class PhotonCameraModule extends PhotonCamera {
    private final String cameraName;
    private final PIDController photonDriveController;
    private final PIDController photonTurnController;
    private final DashTunablePID driveTune;
    private final DashTunablePID turnTune;
    private final DashTunableNumber speedTune;
    private final double cameraHeight;
    private final double cameraPitch;
    private double targetHeight;
    private Pose2d trackedPose = new Pose2d();
    private long lastFoundMillis = System.currentTimeMillis();
    private int aprilTagID = 0;
    private double maxDriveSpeed = PHOTON_DRIVE_MAX_SPEED;


    public void setMaxDriveSpeed(double speed) { this.maxDriveSpeed = speed; }

    public PIDController getPhotonDriveController() { return this.photonDriveController; }
    public PIDController getPhotonTurnController() { return this.photonTurnController; }
    public double getMaxDriveSpeed() { return maxDriveSpeed; }

    public PhotonCameraModule(String name, double height, double pitch) {
        super(name);
        this.cameraName = name;
        this.cameraHeight = height;
        this.cameraPitch = pitch;
        this.photonDriveController = GlobalUtils.generateController(PHOTON_DRIVE_PID);
        this.photonTurnController = GlobalUtils.generateController(PHOTON_TURN_PID);


        if (PHOTON_TUNING_ENABLED) {
            this.driveTune = new DashTunablePID("Photon: Drive PID", PHOTON_DRIVE_PID);
            this.turnTune = new DashTunablePID("Photon: Turn PID", PHOTON_TURN_PID);
            this.speedTune = new DashTunableNumber("Photon: Max Speed", PHOTON_DRIVE_MAX_SPEED);

            speedTune.addConsumer(this::setMaxDriveSpeed);
            driveTune.addConsumer(photonDriveController::setP, photonDriveController::setI, photonDriveController::setD);
            turnTune.addConsumer(photonTurnController::setP, photonTurnController::setI, photonTurnController::setD);
        } else {
            driveTune = null;
            turnTune = null;
            speedTune = null;
        }
    }

    public void setTargetHeight(double height) { this.targetHeight = height; }
    public double getTargetHeight() { return this.targetHeight; }
    public double getCameraHeight() { return this.cameraHeight; }
    public double getCameraPitch() { return this.cameraPitch; }

    public Optional<Pose2d> getTrackedPose() { return Optional.ofNullable(trackedPose); }

    public void update() {
        if (!PHOTON_ENABLED || RobotBase.isSimulation())
            return;

        if (driveTune != null)
            driveTune.update();
        if (turnTune != null)
            turnTune.update();
        if (speedTune != null)
            speedTune.update();

        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            Transform3d targetTransform;
            if (result.getMultiTagResult().estimatedPose.isPresent) {

                targetTransform = result.getMultiTagResult().estimatedPose.best;
            } else {
                PhotonTrackedTarget target = result.getBestTarget();
                targetTransform = target.getBestCameraToTarget();
                aprilTagID = target.getFiducialId();
            }

            trackedPose = new Pose2d(
                    new Translation2d(targetTransform.getX(), targetTransform.getY()),
                    Rotation2d.fromDegrees(targetTransform.getRotation().getAngle())
            );
            lastFoundMillis = System.currentTimeMillis();

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

    public boolean isTargetFound() { return trackedPose != null; }
    public long getLastFoundMillis() { return this.lastFoundMillis; }
    public String getCameraName() { return this.cameraName; }
    public int getAprilTagID() { return this.aprilTagID; }
}
