package frc.robot.subsystems;

import com.pathplanner.lib.util.PIDConstants;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.math.GlobalUtils;
import frc.robot.util.pid.TunableNumber;
import frc.robot.util.pid.TunablePID;

import java.util.*;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

import static com.revrobotics.CANSparkBase.IdleMode.kBrake;
import static com.revrobotics.CANSparkLowLevel.MotorType.kBrushless;

public class BaseSubsystem extends SubsystemBase {
    private final ArrayList<TunableNumber> numberTunes;
    private final HashMap<String, PIDController> controllers;
    private final ArrayList<TunablePID> pidTunes;
    private final String name;
    private final CANSparkMax[] motors;
    private long nextUpdate = System.currentTimeMillis();
    private Runnable dashUpdate = () -> {};
    private boolean tuningEnabled;

    public static ArrayList<String> initSystems = new ArrayList<>();

    public double getTargetSpeed() { return getConstant("Speed"); }
    public void setTargetSpeed(double speed) { setConstant("Speed", speed); }
    public void setTuningEnabled(boolean enabled) { this.tuningEnabled = enabled; }

    public void registerConstant(String name, double value) {
        TunableNumber tune = new TunableNumber(name, value, tuningEnabled);
        numberTunes.add(tune);
    }

    public Supplier<PIDController> registerPID(String name, PIDConstants constants) {
        TunablePID tune = new TunablePID(name, constants, tuningEnabled);
        PIDController controller = GlobalUtils.generateController(constants);
        tune.addConsumer(controller::setP, controller::setI, controller::setD);

        pidTunes.add(tune);
        controllers.put(name, controller);
        return () -> controllers.get(name);
    }

    public double getConstant(String name) {
        for (TunableNumber number : numberTunes) {
            if (number.getName().equalsIgnoreCase(name)) {
                return number.getValue();
            }
        }
        return 0;
    }

    public Optional<PIDController> getPID(String name) {
        for (Map.Entry<String, PIDController> entry : controllers.entrySet()) {
            if (entry.getKey().equalsIgnoreCase(name)) {
                return Optional.of(entry.getValue());
            }
        }
        return Optional.empty();
    }

    @SuppressWarnings("UnusedReturnValue")
    public boolean setConstant(String name, double value) {
        for (TunableNumber number : numberTunes) {
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
        this.numberTunes = new ArrayList<>();
        this.tuningEnabled = tuningEnabled;
        this.pidTunes = new ArrayList<>();
        this.controllers = new HashMap<>();
        registerConstant("Speed", speed);

        if (initSystems.contains(name)) {
            // Do not double-initialize!
            DriverStation.reportWarning("Subsystem: " + name + " attempted re-initialization!", false);
            return;
        }

        if (!ids.isEmpty()) {
            AtomicInteger i = new AtomicInteger(0);
            ids.forEach((id, flip) -> {
                motors[i.get()] = new CANSparkMax(id, kBrushless);
                motors[i.get()].setInverted(flip);
                motors[i.get()].setIdleMode(kBrake);
                motors[i.get()].setSmartCurrentLimit(40);
                GlobalUtils.executeWithDelay(() -> motors[i.get()].burnFlash(), 2000);
                i.set(i.get() + 1);
            });
        }
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

    protected void startAll(double speed) {
        for (CANSparkMax motor : motors)
            motor.set(speed);
    }
    protected void stopAll() { startAll(0); }
    protected void startAll() { startAll(getConstant("Speed")); }
    protected boolean setIndex(int idx, double speed) {
        if (idx > motors.length-1)
            return false;
        motors[idx].set(speed);
        return true;
    }

    @Override
    public void periodic() {
        if (System.currentTimeMillis() >= nextUpdate) {
            if (tuningEnabled) {
                for (TunableNumber tune : numberTunes)
                    tune.update();
                for (TunablePID pidTune : pidTunes)
                    pidTune.update();
                for (int i = 0; i < motors.length; i++)
                    SmartDashboard.putNumber(name + "/Motor " + i + " Amps", motors[i].getOutputCurrent());
            }
            if (dashUpdate != null)
                dashUpdate.run();

            nextUpdate = System.currentTimeMillis() + 1000;
        }
    }
}
