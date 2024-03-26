package frc.robot.subsystems;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.TunableNumber;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class BaseSubsystem extends SubsystemBase {
    private final TunableNumber speedTune;
    private final String name;
    private final CANSparkMax[] motors;
    private long nextUpdate = System.currentTimeMillis();
    private double autoSpeed;
    private double speed;
    private boolean tuningEnabled;

    public static ArrayList<String> initSystems = new ArrayList<>();

    public double getTargetSpeed() { return this.speed; }
    public void setTargetSpeed(double speed) { this.speed = speed; }
    public void setTuningEnabled(boolean enabled) { this.tuningEnabled = enabled; }

    public BaseSubsystem(String name,
                         double speed,
                         boolean tuningEnabled,
                         Map<Integer, Boolean> ids) {
        this.name = name;
        this.motors = new CANSparkMax[ids.size()];
        this.speedTune = new TunableNumber(name, speed, tuningEnabled);
        this.tuningEnabled = tuningEnabled;

        speedTune.addConsumer(this::setTargetSpeed);
        speedTune.setEnabled(tuningEnabled);

        if (initSystems.contains(name)) {
            // Do not double-initialize!
            DriverStation.reportWarning("Subsystem: " + name + " attempted re-initialization!", false);
            return;
        }

        AtomicInteger i = new AtomicInteger(0);
        ids.forEach((id, flip) -> {
            motors[i.get()] = new CANSparkMax(id, kBrushless);
            motors[i.get()].setInverted(flip);
            motors[i.get()].setIdleMode(kBrake);
            motors[i.get()].setSmartCurrentLimit(40);
            GlobalUtils.executeWithDelay(() -> motors[i.get()].burnFlash(), 2000);
            i.set(i.get()+1);
        });
    }

    public double getRPM() {
        double sum = 0;
        for (CANSparkMax motor : motors) {
            RelativeEncoder encoder = motor.getEncoder();
            sum += Math.abs(encoder.getVelocity());
        }
        return sum/ motors.length;
    }

    public double getAutoSpeed() { return this.autoSpeed; }
    public void setAutoSpeed(double autoSpeed) { this.autoSpeed = autoSpeed; }
    public boolean isTuningEnabled() { return this.tuningEnabled; }

    public void stop() {
        for (CANSparkMax motor : motors)
            motor.stopMotor();
    }

    public void startNormal() {
        for (CANSparkMax motor : motors)
            motor.set(speed);
    }

    public void startReverse() {
        for (CANSparkMax motor : motors)
            motor.set(speed);
    }

    @Override
    public void periodic() {
        if (System.currentTimeMillis() >= nextUpdate && tuningEnabled) {
            speedTune.update();
            for (int i = 0; i < motors.length; i++) {
                SmartDashboard.putNumber(name + "/Motor " + i + " Amps", motors[i].getOutputCurrent());
            }
            nextUpdate = System.currentTimeMillis() + 1000;
        }
    }
}
