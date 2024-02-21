package frc.robot.util.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.pid.DashTunableNumber;
import frc.robot.util.pid.DashTunablePID;
import frc.robot.util.pid.PIDConstantsAK;

import java.sql.Driver;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import static frc.robot.Constants.Chassis.*;
import static frc.robot.Constants.Debug.PHOTON_ENABLED;
import static frc.robot.Constants.Debug.PHOTON_TUNING_ENABLED;

public class PhotonCameraModule extends PhotonCamera {
    private final String cameraName;
    private final PIDController driveController;
    private final PIDController turnController;
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

    public double getMaxDriveSpeed() { return maxDriveSpeed; }

    public void setMaxDriveSpeed(double speed) {
        this.maxDriveSpeed = speed;
    }

    public PIDController getDriveController() { return this.driveController; }
    public PIDController getTurnController() { return this.turnController; }

    public PhotonCameraModule(String name, double height, double pitch) {
        super(name);
        this.cameraName = name;
        this.cameraHeight = height;
        this.cameraPitch = pitch;

        this.driveController = PIDConstantsAK.generateController(PHOTON_DRIVE_PID);
        this.turnController = PIDConstantsAK.generateController(PHOTON_TURN_PID);


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


    public void setTargetHeight(double height) { this.targetHeight = height; }
    public double getTargetHeight() { return this.targetHeight; }
    public double getCameraHeight() { return this.cameraHeight; }
    public double getCameraPitch() { return this.cameraPitch; }

    public Optional<Pose2d> getTrackedPose() { return Optional.ofNullable(trackedPose); }

    public void update() {
        if (!PHOTON_ENABLED || RobotBase.isSimulation())
            return;

        driveTune.update();
        turnTune.update();
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
            SmartDashboard.putString("Photon Pose", trackedPose.toString());
        } else {
            if (System.currentTimeMillis() >= lastFoundMillis+500) {
                trackedPose = null;
                SmartDashboard.putString("Photon Pose", "NONE");
            }
        }
    }

    public boolean isTargetFound() { return trackedPose != null; }
    public long getLastFoundMillis() { return this.lastFoundMillis; }
    public String getCameraName() { return this.cameraName; }
    public int getAprilTagID() { return this.aprilTagID; }
}
