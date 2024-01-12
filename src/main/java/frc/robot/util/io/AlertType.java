package frc.robot.util.io;

import java.util.ArrayList;
import java.util.List;

public enum AlertType {
    WARNING("warnings"),
    ERROR("errors");

    private final String prefix;

    public static final AlertType[] DASHBOARD_ORDER = new AlertType[]{WARNING, ERROR};

    AlertType(String prefix) { this.prefix = prefix; }
    public String getPrefix() { return this.prefix; }
}
