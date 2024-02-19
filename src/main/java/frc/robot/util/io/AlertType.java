package frc.robot.util.io;

public enum AlertType {
    WARNING("warnings"),
    ERROR("errors"),
    INFO("infos");

    private final String prefix;

    public static final AlertType[] DASHBOARD_ORDER = new AlertType[]{ERROR, WARNING, INFO};

    AlertType(String prefix) { this.prefix = prefix; }
    public String getPrefix() { return this.prefix; }
}
