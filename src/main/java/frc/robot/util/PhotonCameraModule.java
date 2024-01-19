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

import static frc.robot.Constants.Chassis.PHOTON_PID_CONFIG;
import static frc.robot.Constants.LooperConfig.STRING_ODOMETRY_NAME;
import static frc.robot.Constants.LooperConfig.STRING_PERIODIC_NAME;

public class PhotonCameraModule extends PhotonCamera implements Subsystem {
    public static final int CAMERA_BUFFER_MILLIS = 500;

    private final double cameraHeight;
    private final double cameraPitch;
    private final String cameraName;
    private double targetHeight;
    private final PIDController pidController;
    private Pose2d trackedPose;
    private long lastFoundMillis = System.currentTimeMillis();
    private final DashTunablePID dashTune;
    private int aprilTagID = 0;

    public PIDController getController() { return this.pidController; }

    public PhotonCameraModule(String name, double height, double pitch) {
        super(name);
        this.cameraName = name;
        this.cameraHeight = height;
        this.cameraPitch = pitch;
        this.pidController = new PIDController(
                PHOTON_PID_CONFIG.kP,
                PHOTON_PID_CONFIG.kI,
                PHOTON_PID_CONFIG.kD
        );
        this.dashTune = new DashTunablePID("Photon PID", PHOTON_PID_CONFIG);
        dashTune.addConsumer(pidController::setP, pidController::setI, pidController::setD);

        IOManager.addPeriodicIfExists(STRING_ODOMETRY_NAME, dashTune::update);
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
        if (trackedPose == null || System.currentTimeMillis() >= lastFoundMillis + 500) {
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
