package frc.robot.util.pid;

public interface IUpdatable {
    void update();

    static void updateAll(IUpdatable... objects) {
        if (objects == null)
            return;
        for (IUpdatable obj : objects) {
            if (obj != null)
                obj.update();
        }
    }
}
