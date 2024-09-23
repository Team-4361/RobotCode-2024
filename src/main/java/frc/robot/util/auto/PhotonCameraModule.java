package frc.robot.util.auto;

import com.pathplanner.lib.util.PIDConstants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.subsystems.base.BaseSubsystem;
import frc.robot.subsystems.base.SubsystemConfig;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

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
    private Transform2d targetPose;
    private AprilTagID aprilTag;
    private long lastFoundMillis;
    private long nextCheckMillis;
    private int selectedIndex = 0;

    public String getCameraName() { return camera.getName(); }

    public Optional<AprilTagID> getAprilTag() { return Optional.ofNullable(aprilTag); }
    public Optional<Transform2d> getTrackedDistance() { return Optional.ofNullable(trackedPose); }
    public Optional<Transform2d> getTarget() { return Optional.ofNullable(targetPose); }

    public PIDController getDriveController() { return drivePID; }
    public PIDController getTurnController() { return turnPID; }
    public PipelineOption getPipeline() { return pipelines.get(selectedIndex); }

    public double getMaxDrivePower() { return getConstant(DRIVE_POWER_NAME); }
    public double getMaxTurnPower() { return getConstant(TURN_POWER_NAME); }
    public void setMaxDrivePower(double power) { setConstant(DRIVE_POWER_NAME, power); }
    public void setMaxTurnPower(double power) { setConstant(TURN_POWER_NAME, power); }

    public boolean hasTarget() { return trackedPose != null; }

    public PhotonCameraModule(SubsystemConfig config, Transform3d transform, List<PipelineOption> pipelines) {
        super(config, new HashMap<>());

        this.pipelines = new ArrayList<>();
        this.trackedPose = new Transform2d();
        this.lastFoundMillis = System.currentTimeMillis();
        this.cameraTransform = transform;
        this.pipelines.addAll(pipelines);

        this.drivePID = registerPID(DRIVE_PID_NAME, pipelines.get(0).drivePID());
        this.turnPID = registerPID(TURN_PID_NAME, pipelines.get(0).turnPID());

        registerConstant(TIMEOUT_NAME, 500);
        registerConstant(DRIVE_POWER_NAME, PHOTON_DRIVE_MAX_SPEED);
        registerConstant(TURN_POWER_NAME, PHOTON_TURN_MAX_SPEED);

        this.camera = isEnabled() ? new PhotonCamera(config.name()) : null;

        turnPID.enableContinuousInput(0, 180);

        setDashUpdate(() -> {
            SmartDashboard.putString(getCameraName() + "/Photon Pose", trackedPose == null ? "NONE" : trackedPose.toString());
            SmartDashboard.putBoolean(getCameraName() + "/Rot Target", atRotationTarget());
            SmartDashboard.putBoolean(getCameraName() + "/Object Visible", isDetected());
        });
    }

    @SuppressWarnings("unusedReturn")
    public PhotonCameraModule addPipeline(PipelineOption pipeline) {
        pipelines.add(pipeline);
        return this;
    }

    public boolean setPipeline(PipelineOption pipe) { return setPipeline(pipe.name()); }

    public boolean setPipeline(String name) {
        for (int i = 0; i < pipelines.size(); i++) {
            if (pipelines.get(i).name().equalsIgnoreCase(name)) {
                setPipeline(i);
                return true;
            }
        }
        return false;
    }

    public void setTarget(Transform2d target) { this.targetPose = target; }

    public boolean atRotationTarget() {
        if (targetPose == null || trackedPose == null)
            return false;
        return MathUtil.isNear(
                trackedPose
                        .getRotation()
                        .getDegrees()%180,
                targetPose
                        .getRotation()
                        .getDegrees()%180,
                2);
    }

    public boolean atTarget() {
        if (targetPose == null || trackedPose == null)
            return false;
        return MathUtil.isNear(trackedPose.getX(), targetPose.getX(), 0.1) &&
                MathUtil.isNear(trackedPose.getY(), targetPose.getY(), 0.1) &&
                atRotationTarget();
    }

    public boolean isDetected() { return trackedPose != null; }

    public ChassisSpeeds getNextTargetSpeeds() {
        if (trackedPose == null || targetPose == null)
            return new ChassisSpeeds();

        double mXY = getMaxDrivePower();
        double mO = getMaxTurnPower();
        double jX = -MathUtil.clamp(drivePID.calculate(trackedPose.getX(), targetPose.getX()), -mXY, mXY);
        double jY = -MathUtil.clamp(drivePID.calculate(trackedPose.getY(), targetPose.getY()), -mXY, mXY);
        double jO = -MathUtil.clamp(
                turnPID.calculate(
                        trackedPose.getRotation().getDegrees(),
                        targetPose.getRotation().getDegrees()
                ),
                -mO,
                mO
        );

        return new ChassisSpeeds(
                jX * Robot.swerve.getMaximumVelocity(),
                jY * Robot.swerve.getMaximumVelocity(),
                jO * Robot.swerve.getMaximumAngularVelocity()
        );
    }

    @SuppressWarnings("UnusedReturnValue")
    public boolean setPipeline(int index) {
        if (index > pipelines.size() - 1 || index < 0)
            return false;
        selectedIndex = index;
        if (camera != null)
            camera.setPipelineIndex(index);

        // Update the Drive and Turn PID constants with their updated values.
        PipelineOption pipeline = getPipeline();
        PIDConstants dpConstants = pipeline.drivePID();
        PIDConstants tpConstants = pipeline.turnPID();

        if (camera != null && index != camera.getPipelineIndex()) {
            drivePID.setPID(dpConstants.kP, dpConstants.kI, dpConstants.kD);
            turnPID.setPID(tpConstants.kP, tpConstants.kI, tpConstants.kD);

            syncDashboardPID(DRIVE_PID_NAME, drivePID);
            syncDashboardPID(TURN_PID_NAME, turnPID);
            trackedPose = null;
            nextCheckMillis = System.currentTimeMillis() + (long)getConstant(TIMEOUT_NAME);
        }
        return true;
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!isEnabled() || RobotBase.isSimulation())
            return;

        if (System.currentTimeMillis() < nextCheckMillis)
            return;

        try {
            if (camera == null) {
                trackedPose = null;
                return;
            }

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
                    // We are using the shape method. Do the theorem to calculate

                    Rotation3d camRotation = cameraTransform.getRotation();
                    fX = target.getPitch();
                    fY = 0;
                    fO = Rotation2d.fromDegrees(-target.getYaw());
                    //fO = Rotation2d.fromDegrees(target.getYaw());

                    /*
                    fX = PhotonUtils.calculateDistanceToTargetMeters(
                            cameraTransform.getZ(),
                            pipe.targetHeight(),
                            cameraTransform
                                    .getRotation()
                                    .getY(),
                            Units.degreesToRadians(target.getPitch())
                    );
                    fO = Rotation2d.fromDegrees(target.getYaw());
                    fY = 0;

                     */
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
        } catch (Exception exception) {
            trackedPose = null;
        }
        nextCheckMillis = System.currentTimeMillis();
    }
}