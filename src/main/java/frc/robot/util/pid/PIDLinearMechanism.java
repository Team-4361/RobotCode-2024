package frc.robot.util.pid;


import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.math.PeakMotorDistance;
import frc.robot.util.motor.FRCSparkMax;
import frc.robot.util.motor.IMotorModel;

public class PIDLinearMechanism extends PIDMechanismBase {
    private final PeakMotorDistance maxDistance;
    private final DistanceUnit unit;
    private boolean tuneMode;

    public enum DistanceUnit { FEET, METERS, INCHES }

    public void setDistanceTuningEnabled(boolean enable) {
        setPIDControlSupplier(()->!enable);
        tuneMode = enable;
    }

    public boolean getTuneMode() { return tuneMode; }

    private double convertUnits(double motorRotations) {
        double distanceMeters = maxDistance.rotationToMeters(motorRotations);

        return switch (unit) {
            case FEET -> Units.metersToFeet(distanceMeters);
            case INCHES -> Units.metersToInches(distanceMeters);
            default -> distanceMeters;
        };
    }

    /**
     * Constructs a new {@link PIDLinearMechanism}.
     *
     * @param motorId        The {@link FRCSparkMax} motor ID to use.
     * @param constants      The {@link PIDConstantsAK} to use.
     * @param kS             The {@link SimpleMotorFeedforward} kS constant.
     * @param kV             The {@link SimpleMotorFeedforward} kV constant.
     * @param kA             The {@link SimpleMotorFeedforward} kA constant.
     * @param model          The {@link IMotorModel} of the {@link FRCSparkMax} motor.
     * @param moduleName     The {@link String} module name
     * @param tuningEnabled  If PID {@link SmartDashboard} tuning is enabled.
     * @param unit           The conversion unit of the {@link PIDLinearMechanism}. <b>MUST BE CONSISTENT!</b>
     * @param maxDistance    The {@link PeakMotorDistance} used by the {@link PIDLinearMechanism}.
     */
    public PIDLinearMechanism(int motorId,
                              PIDConstantsAK constants,
                              double kS,
                              double kV,
                              double kA,
                              IMotorModel model,
                              String moduleName,
                              boolean tuningEnabled,
                              DistanceUnit unit,
                              PeakMotorDistance maxDistance) {
        super(motorId, constants, kS, kV, kA, model, moduleName, tuningEnabled, false);
        this.unit = unit;
        this.maxDistance = maxDistance;

        setForwardLimit(convertUnits(maxDistance.getRotation())); // set the maximum forward limit to the peak.
    }

    /**
     * @param motorRotations The motor rotations as reported by the {@link Encoder}.
     * @return The current value which should be used for current positional based control. It MUST be in the
     * <b>SAME UNIT</b> as the Target Position.
     */
    @Override
    protected double getCurrentPosition(double motorRotations) {
        if (tuneMode){
            return motorRotations;
        }
        return convertUnits(maxDistance.rotationToMeters(motorRotations));
    }
}
