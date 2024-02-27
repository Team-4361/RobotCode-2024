package frc.robot.util.motor;

import edu.wpi.first.wpilibj.Servo;

import static frc.robot.Constants.TrapArm.ARM_SERVO_MAX_MM;

public class MotorServo extends Servo {
    private final double maxMM;
    private final double minMM;
    private double servoTargetMM = 0;
    private double servoPositionMM = 0;
    private double lastSpeed = 0;

    public double getTargetMM() { return this.servoTargetMM; }
    public double getDistanceMM() { return servoPositionMM; }

    public void setDistance(double mm) {
        servoTargetMM = mm;
        this.setPosition(Math.max(minMM, servoTargetMM / maxMM));
    }

    public void update() {
        servoPositionMM = getPosition() * ARM_SERVO_MAX_MM;
    }

    @Override
    public void set(double speed) {
        if (speed != lastSpeed) {
            if (speed == 0) {
                super.set(lastSpeed);
            } else {
                super.set(speed);
            }
            lastSpeed = speed;
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
