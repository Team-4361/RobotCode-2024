package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.TunableNumber;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Consumer;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class BaseSubsystem extends SubsystemBase {
    private final ArrayList<TunableNumber> tunes;
    private final String name;
    private final CANSparkMax[] motors;
    private long nextUpdate = System.currentTimeMillis();
    private Runnable dashUpdate = () -> {};
    private double speed;
    private boolean tuningEnabled;

    public static ArrayList<String> initSystems = new ArrayList<>();

    public double getTargetSpeed() { return this.speed; }
    public void setTargetSpeed(double speed) { this.speed = speed; }
    public void setTuningEnabled(boolean enabled) { this.tuningEnabled = enabled; }

    public void registerConstant(String name, double value) {
        TunableNumber tune = new TunableNumber(name, value, tuningEnabled);
        //tune.addConsumer(consumer);
        tunes.add(tune);
    }

    public double getConstant(String name) {
        for (TunableNumber number : tunes) {
            if (number.getName().equalsIgnoreCase(name)) {
                return number.getValue();
            }
        }
        return 0;
    }

    public boolean setConstant(String name, double value) {
        for (TunableNumber number : tunes) {
            if (number.getName().equalsIgnoreCase(name)) {
                number.setValue(value);
                return true;
            }
        }
        return false;
    }

    public BaseSubsystem(String name,
                         double speed,
                         boolean tuningEnabled,
                         Map<Integer, Boolean> ids) {
        this.name = name;
        this.motors = new CANSparkMax[ids.size()];
        this.tunes = new ArrayList<>();
        this.tuningEnabled = tuningEnabled;
        registerConstant("Speed", speed);

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

    public void setDashUpdate(Runnable runnable) { this.dashUpdate = runnable; }

    public double getRPM() {
        double sum = 0;
        for (CANSparkMax motor : motors) {
            RelativeEncoder encoder = motor.getEncoder();
            sum += Math.abs(encoder.getVelocity());
        }
        return sum/ motors.length;
    }

    public boolean isTuningEnabled() { return this.tuningEnabled; }

    public void stop() {
        for (CANSparkMax motor : motors)
            motor.stopMotor();
    }

    public void start(double speed) {
        for (CANSparkMax motor : motors)
            motor.set(speed);
    }

    public void startReverse(double speed) { start(-Math.abs(speed)); }
    public void startReverse() { startReverse(speed); }
    public void start() { start(speed); }

    public void startIndex(double index, )

    @Override
    public void periodic() {
        if (System.currentTimeMillis() >= nextUpdate) {
            if (tuningEnabled) {
                for (TunableNumber tune : tunes)
                    tune.update();
                for (int i = 0; i < motors.length; i++)
                    SmartDashboard.putNumber(name + "/Motor " + i + " Amps", motors[i].getOutputCurrent());
            }
            if (dashUpdate != null)
                dashUpdate.run();

            nextUpdate = System.currentTimeMillis() + 1000;
        }
    }
}
