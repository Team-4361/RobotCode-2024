package frc.robot.util.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.io.IOManager;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.Constants.Chassis.CHASSIS_MODE;
import static frc.robot.Constants.Debug.PHOTON_ENABLED;
import static frc.robot.Constants.Debug.PHOTON_TUNING_ENABLED;

public class PhotonCameraModule extends PhotonCamera implements Subsystem {
    private final String cameraName;
    private final PIDController driveController;
    private final PIDController turnController;
    private final double cameraHeight;
    private final double cameraPitch;
    private double targetHeight;
    private Pose2d trackedPose = new Pose2d();
    private long lastFoundMillis = System.currentTimeMillis();
    private int aprilTagID = 0;

    public PIDController getDriveController() { return this.driveController; }
    public PIDController getTurnController() { return this.turnController; }

    public PhotonCameraModule(String name, double height, double pitch) {
        super(name);
        this.cameraName = name;
        this.cameraHeight = height;
        this.cameraPitch = pitch;

        this.driveController = new PIDController(
                CHASSIS_MODE.getAutoDrivePID().kP,
                CHASSIS_MODE.getAutoDrivePID().kI,
                CHASSIS_MODE.getAutoDrivePID().kD
        );
        this.turnController = new PIDController(
                CHASSIS_MODE.getAutoTurnPID().kP,
                CHASSIS_MODE.getAutoTurnPID().kI,
                CHASSIS_MODE.getAutoTurnPID().kD
        );

        if (PHOTON_TUNING_ENABLED) {
            IOManager.initPIDTune("Photon: Drive PID", driveController);
            IOManager.initPIDTune("Photon: Turn PID", turnController);
        }

        CommandScheduler.getInstance().registerSubsystem(this);
    }


    public void setTargetHeight(double height) {
        this.targetHeight = height;
    }

    public double getTargetHeight() { return this.targetHeight; }
    public double getCameraHeight() { return this.cameraHeight; }
    public double getCameraPitch() { return this.cameraPitch; }

    public Optional<Pose2d> getTrackedPose() { return Optional.ofNullable(trackedPose); }

    @Override
    public void periodic() {
        if (!PHOTON_ENABLED)
            return;
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
