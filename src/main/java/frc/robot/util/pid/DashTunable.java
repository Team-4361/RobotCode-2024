package frc.robot.util.pid;

import java.util.ArrayList;
import java.util.EnumSet;
import java.util.function.Consumer;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEvent.Kind;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;

public abstract class DashTunable<T> implements Sendable {
    public T value;

    public ArrayList<Consumer<T>> listeners = new ArrayList<>();
    public final GenericEntry entry;
    public final String name;

    protected abstract void setNT(T value);

    protected abstract T getFromNT();

    protected DashTunable(T value, String name) {
        this.value = value;
        this.name = name;

        NetworkTable nt = NetworkTableInstance.getDefault().getTable("Tuning");
        entry = nt.getTopic(name).getGenericEntry();
        setNT(value);

        nt.addListener(name, EnumSet.of(Kind.kValueAll), (table, key, event) -> {
            MatchType matchType = DriverStation.getMatchType();
            if (matchType == MatchType.None
                    || matchType == MatchType.Practice
                    || DriverStation.isTest()) {
                this.value = getFromNT();
                this.listeners.forEach(o -> o.accept(value));
            }
        });
    }

    public static DashTunable<Double> of(double value, String name) {
        return new DashTunable<>(value, name) {
            @Override
            protected Double getFromNT() {
                return entry.getDouble(value);
            }

            @Override
            protected void setNT(Double value) {
                entry.setDouble(value);
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addDoubleProperty("value", this::get, this::setNT);
            }
        };
    }

    public static DashTunable<Boolean> of(boolean value, String name) {
        return new DashTunable<>(value, name) {
            @Override
            protected Boolean getFromNT() {
                return entry.getBoolean(value);
            }

            @Override
            protected void setNT(Boolean value) {
                entry.setBoolean(value);
            }

            @Override
            public void initSendable(SendableBuilder builder) {
                builder.addBooleanProperty("value", this::get, this::setNT);
            }
        };
    }

    public DashTunable<T> addListener(Consumer<T> updater) {
        listeners.add(updater);
        return this;
    }

    public T get() { return value; }
}