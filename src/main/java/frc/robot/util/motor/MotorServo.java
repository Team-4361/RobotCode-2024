package frc.robot.util.motor;

import edu.wpi.first.wpilibj.Servo;

public class MotorServo extends Servo {
    private final double maxMM;
    private final double minMM;
    private double servoTargetMM = 0;
    private double servoPositionMM = 0;
    private double lastSpeed = 0;
    private long nextManualUpdate = System.currentTimeMillis();

    public double getTargetMM() { return this.servoTargetMM; }
    public double getDistanceMM() { return servoPositionMM; }

    public void setDistance(double mm) {
        if (mm > maxMM)
            mm = maxMM;
        if (mm < minMM)
            mm = minMM;
        servoTargetMM = mm;
        this.setPosition(mm / maxMM);
    }

    public void update() {
        servoPositionMM = getPosition() * maxMM;
    }

    @Override
    public void set(double speed) {
        if (speed == 0)
            return;
        if (System.currentTimeMillis() >= nextManualUpdate) {
            setDistance(getDistanceMM() + speed);
            nextManualUpdate = System.currentTimeMillis() + 20;
        }
    }

    public MotorServo(int channel,
                      int max,
                      int deadbandMax,
                      int center,
                      int deadbandMin,
                      int min,
                      double minMM,
                      double maxMM) {

        super(channel);
        this.maxMM = maxMM;
        this.minMM = minMM;

        setBoundsMicroseconds(max, deadbandMax, center, deadbandMin, min);
        setDistance(minMM);
    }
}
