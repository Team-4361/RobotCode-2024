package frc.robot.util.swerve;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import java.util.HashMap;
import java.util.Map;

import static frc.robot.Constants.Chassis.*;

public class SwerveChassis {
    private static final Translation2d SWERVE_FL_POSITION =
            new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final Translation2d SWERVE_FR_POSITION =
            new Translation2d(SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final Translation2d SWERVE_BL_POSITION =
            new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final Translation2d SWERVE_BR_POSITION =
            new Translation2d(-SWERVE_CHASSIS_SIDE_LENGTH / 2, -SWERVE_CHASSIS_SIDE_LENGTH / 2);

    private static final SwerveDriveKinematics SWERVE_KINEMATICS =
            new SwerveDriveKinematics(
                    SWERVE_FL_POSITION,
                    SWERVE_FR_POSITION,
                    SWERVE_BL_POSITION,
                    SWERVE_BR_POSITION
            );
    
    private static final String NAME_FL = "FL";
    private static final String NAME_FR = "FR";
    private static final String NAME_BL = "BL";
    private static final String NAME_BR = "BR";

    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    public SwerveChassis(SwerveModule frontLeft,
                         SwerveModule frontRight,
                         SwerveModule backLeft,
                         SwerveModule backRight) {
        this.frontRight = frontRight;
        this.frontLeft = frontLeft;
        this.backRight = backRight;
        this.backLeft = backLeft;
        updateDashboard();
    }

    private void updateDashboard() {
        frontLeft.updateDashboard(NAME_FL);
        frontRight.updateDashboard(NAME_FR);
        backLeft.updateDashboard(NAME_BL);
        backRight.updateDashboard(NAME_BR);
    }

    public SwerveModule getFrontLeft() { return frontLeft; }
    public SwerveModule getFrontRight() { return frontRight; }
    public SwerveModule getBackLeft() { return backLeft; }
    public SwerveModule getBackRight() { return backRight; }

    public SwerveDriveKinematics getSwerveKinematics() { return SWERVE_KINEMATICS; }

    public HashMap<String, SwerveModuleState> getStates() {
        return new HashMap<>(Map.of(
                "FL", getFrontLeft().getState(),
                "BL", getBackLeft().getState(),
                "FR", getFrontRight().getState(),
                "BR", getBackRight().getState()
        ));
    }

    public SwerveModulePosition[] getPositions() {
        return new SwerveModulePosition[] {
                getFrontLeft().getPosition(),
                getFrontRight().getPosition(),
                getBackLeft().getPosition(),
                getBackRight().getPosition()
        };
    }

    public void setStates(SwerveModuleState[] states, boolean isClosedLoop) {
        frontLeft.setState(states[0], isClosedLoop);
        frontRight.setState(states[1], isClosedLoop);
        backLeft.setState(states[2], isClosedLoop);
        backRight.setState(states[3], isClosedLoop);
    }

    public void drive(ChassisSpeeds speeds, boolean isClosedLoop) {
        setStates(SWERVE_KINEMATICS.toSwerveModuleStates(speeds), isClosedLoop);
        updateDashboard();
    }

    public void resetDriveEncoders() {
        frontLeft.reset();
        frontRight.reset();
        backLeft.reset();
        backRight.reset();
    }

    public SwerveChassis getSwerveChassis(){
        return this;
    }
}