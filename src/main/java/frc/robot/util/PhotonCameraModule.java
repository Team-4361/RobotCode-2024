package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.io.IOManager;
import frc.robot.util.pid.DashTunablePID;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Optional;

import static frc.robot.Constants.Chassis.PHOTON_DRIVE_PID_CONSTANTS;
import static frc.robot.Constants.Chassis.TURN_PID_CONFIG;
import static frc.robot.Constants.LooperConfig.STRING_ODOMETRY_NAME;

public class PhotonCameraModule extends PhotonCamera implements Subsystem {
    public static final int CAMERA_BUFFER_MILLIS = 500;

    private final double cameraHeight;
    private final double cameraPitch;
    private final String cameraName;
    private double targetHeight;
    private final PIDController driveController;
    private final PIDController turnController;
    private Pose2d trackedPose;
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
                PHOTON_DRIVE_PID_CONSTANTS.kP,
                PHOTON_DRIVE_PID_CONSTANTS.kI,
                PHOTON_DRIVE_PID_CONSTANTS.kD
        );
        this.turnController = new PIDController(
                TURN_PID_CONFIG.kP,
                TURN_PID_CONFIG.kI,
                TURN_PID_CONFIG.kD
        );
        DashTunablePID dashTune = new DashTunablePID("Photon Drive PID", PHOTON_DRIVE_PID_CONSTANTS);
        DashTunablePID turnTune = new DashTunablePID("Photon Turn PID", TURN_PID_CONFIG);

        dashTune.addConsumer(driveController::setP, driveController::setI, driveController::setD);
        turnTune.addConsumer(turnController::setP, turnController::setI, turnController::setD);

        IOManager.addPeriodicIfExists(STRING_ODOMETRY_NAME, () -> {
            dashTune.update();
            turnTune.update();
        });
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
        // Current pose is null OR the millisecond count is too high. Recalculate it.
        PhotonPipelineResult result = getLatestResult();
        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();
            Transform3d targetTransform = target.getBestCameraToTarget();

            if (targetTransform.getX() == 0 || targetTransform.getY() == 0) {
                trackedPose = new Pose2d(
                        new Translation2d(
                                PhotonUtils.calculateDistanceToTargetMeters(
                                        cameraHeight,
                                        targetHeight,
                                        cameraPitch,
                                        target.getPitch()), 0),
                        new Rotation2d(target.getYaw())
                );
            } else {
                trackedPose = new Pose2d(
                        new Translation2d(targetTransform.getX(), targetTransform.getY()),
                        Rotation2d.fromDegrees(target.getYaw())
                );
                aprilTagID = target.getFiducialId();
            }
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
