package frc.robot.util.motor;

public class TimedDigitalInput extends DigitalInput {
  private long activatedMs = 0;

  public void update() {
    if (get()) {
      if (activatedMs == 0)
          activatedMs = System.currentTimeMillis();
    } else {
      activatedMs = 0;
    }
  }

  public long getActivatedDuration() {
    return (activatedMs == 0) ? 0 : System.currentTimeMillis() - activatedMs;
  }

  public TimedDigitalInput(int id) {
    super(id);
  }
}
