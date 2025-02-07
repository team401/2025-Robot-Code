import java.util.LinkedList;
import java.util.Queue;

import frc.robot.constants.AutoPath.Action;

public class StrategyManager {
    private Queue<Action> actions = null;

    public StrategyManager () {
        actions = new LinkedList<>();
    }

    public void addAction(Action action) {
        actions.add(action);
    }

    public Action getNextAction() {
        return actions.remove();
    }
}