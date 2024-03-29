package frc.robot.util.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.base.BaseSubsystem;
import frc.robot.subsystems.base.SubsystemConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;
import java.util.function.Supplier;

import static frc.robot.Constants.Chassis.*;

@SuppressWarnings("unused")
public class PhotonCameraModule extends BaseSubsystem {

    protected static final String DRIVE_PID_NAME = "DrivePID";
    protected static final String TURN_PID_NAME = "TurnPID";
    protected static final String DRIVE_POWER_NAME = "DrivePower";
    protected static final String TURN_POWER_NAME = "TurnPower";
    protected static final String TIMEOUT_NAME = "Timeout";

    private final PIDController drivePID;
    private final PIDController turnPID;
    private final ArrayList<PipelineOption> pipelines;

    private final Transform3d cameraTransform;
    private final PhotonCamera camera;

    private Transform2d trackedPose;
    private AprilTagID aprilTag;
    private long lastFoundMillis;
    private int selectedIndex = 0;

    public String getCameraName() { return camera.getName(); }

    public Optional<AprilTagID> getAprilTag() { return Optional.ofNullable(aprilTag); }
    public Optional<Transform2d> getTrackedDistance() { return Optional.ofNullable(trackedPose); }

    public PIDController getDriveController() { return drivePID; }
    public PIDController getTurnController() { return turnPID; }
    public PipelineOption getPipeline() { return pipelines.get(selectedIndex); }

    public double getMaxDrivePower() { return getConstant(DRIVE_POWER_NAME); }
    public double getMaxTurnPower() { return getConstant(TURN_POWER_NAME); }

    public void setMaxDrivePower(double power) { setConstant(DRIVE_POWER_NAME, power); }
    public void setMaxTurnPower(double power)  { setConstant(TURN_POWER_NAME, power);  }

    // TODO: implement timeout!!!

    public PhotonCameraModule(SubsystemConfig config, Transform3d transform) {
        super(config, new HashMap<>());

        this.camera = new PhotonCamera(config.name());
        this.pipelines = new ArrayList<>();

        this.drivePID = registerPID(DRIVE_PID_NAME, PHOTON_DRIVE_PID);
        this.turnPID = registerPID(TURN_PID_NAME, PHOTON_TURN_PID);
        this.cameraTransform = transform;
        this.trackedPose = new Transform2d();
        this.lastFoundMillis = System.currentTimeMillis();

        registerConstant(TIMEOUT_NAME, 500);
        registerConstant(DRIVE_POWER_NAME, PHOTON_DRIVE_MAX_SPEED);
        registerConstant(TURN_POWER_NAME, PHOTON_TURN_MAX_SPEED);

        turnPID.enableContinuousInput(-180, 180);

        setDashUpdate(() -> {
            SmartDashboard.putString("Photon Pose", trackedPose == null ? "NONE" : trackedPose.toString());
        });
    }

    @SuppressWarnings("unusedReturn")
    public PhotonCameraModule addPipeline(PipelineOption pipeline) {
        pipelines.add(pipeline);
        return this;
    }

    public boolean setPipeline(PipelineOption pipe) { return setPipeline(pipe.name()); }

    public boolean setPipeline(String name) {
        for (int i=0; i<pipelines.size(); i++) {
            if (pipelines.get(i).name().equalsIgnoreCase(name)) {
                selectedIndex = i;
                camera.setPipelineIndex(i);
                return true;
            }
        }
        return false;
    }

    public boolean setPipeline(int index) {
        if (index > pipelines.size()-1 || index < 0)
            return false;
        selectedIndex = index;
        camera.setPipelineIndex(index);
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!isEnabled() || RobotBase.isSimulation())
            return;

        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            // If we are using AprilTags, calculate the transform from the "cameraToTarget"
            PipelineOption pipe = getPipeline();
            double fX, fY;
            Rotation2d fO;

            if (pipe.isAprilTag()) {
                AprilTagID.fromID(target.getFiducialId())
                        .ifPresent(o -> aprilTag = o);

                Transform3d transform = target.getBestCameraToTarget();
                fX = transform.getX();
                fY = transform.getY();
                fO = transform.getRotation().toRotation2d();
            } else {
                // We are using the shape method. Do the theorem to calculate.
                /*
                fX = PhotonUtils.calculateDistanceToTargetMeters(
                        cameraTransform.getZ(),
                        pipe.targetHeight(),
                        cameraTransform
                                .getRotation()
                                .toRotation2d()
                                .getRadians(),
                        Units.degreesToRadians(target.getPitch())
                );
                 */
                fX = target.getPitch();
                fY = -target.getYaw();
                fO = Rotation2d.fromDegrees(0);
                //fO = Rotation2d.fromDegrees(target.getYaw());
                aprilTag = null;
            }

            lastFoundMillis = System.currentTimeMillis();
            trackedPose = new Transform2d(
                    new Translation2d(fX, fY),
                    fO
            );
        } else {
            if (System.currentTimeMillis() >=
                    lastFoundMillis + getConstant(TIMEOUT_NAME)) {
                trackedPose = null;
            }
        }
    }
}