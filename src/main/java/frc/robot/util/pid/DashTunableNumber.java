package frc.robot.util.pid;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.function.Consumer;

public class DashTunableNumber {
    private final String name;
    private final String dashString;
    private final ArrayList<Consumer<Double>> consumers;
    private double value;

    public DashTunableNumber(String name, double initialValue, boolean usePrefix) {
        this.name = name;
        this.value= initialValue;
        if (usePrefix)
            this.dashString = name + ": Number";
        else
            this.dashString = name;
        this.consumers = new ArrayList<>();
        SmartDashboard.putNumber(dashString, value);
    }
    public DashTunableNumber(String name, double initialValue){
        this(name, initialValue, true);
    }

    // Add a method to add a Consumer which accepts "Consumer<Double>" parameter
    // and uses the "add" method to put it into the arraylist.
    public void addConsumer(Consumer<Double> consumers){
        this.consumers.add(consumers);
    }

    // add getter methods for all final variables and "value"
    // do not add set method for "value"

    public String getName(){
        return this.name;
    }
    public String getDashString(){
        return this.dashString;
    }
    public ArrayList<Consumer<Double>> getConsumers(){
        return this.consumers;
    }
    public double getValue(){
        return this.value;
    }
    public void update() {
        // add smartdashboard logic
        value = SmartDashboard.getNumber(dashString, value);
        consumers.forEach(o -> o.accept(value));
    }
}
