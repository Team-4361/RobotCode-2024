package frc.robot.util.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.util.swerve.config.GyroIO;

public class GyroIONavX1 implements GyroIO {
    private final AHRS gyro;

    public GyroIONavX1(SPI.Port port) {
        gyro = new AHRS(port);
        gyro.reset();
    }

    @Override
    public void updateInputs(GyroIOInputs inputs) {
        inputs.connected = gyro.isConnected();
        inputs.yawPosition = gyro.getRotation2d();
        inputs.yawVelocityRadPerSec = Units.degreesToRadians(gyro.getRate());
        inputs.pitchPosition = Rotation2d.fromDegrees(gyro.getPitch());
        inputs.rollPosition = Rotation2d.fromDegrees(gyro.getRoll());
        inputs.isCalibrating = gyro.isCalibrating();
    }

    @Override
    public void reset() {
        gyro.reset();
    }


}
