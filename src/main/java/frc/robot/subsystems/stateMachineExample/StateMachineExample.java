package frc.robot.subsystems.stateMachineExample;

import coppercore.controls.state_machine.StateMachine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.stateMachineExample.state_machine.ExampleStateContainer;
import frc.robot.subsystems.stateMachineExample.state_machine.ExampleStateMachine;

public class StateMachineExample extends SubsystemBase {

    private StateMachine<ExampleStateContainer, ExampleActions> stateMachine;

    private ExampleActions currentAction = ExampleActions.IDLE;

    public static enum ExampleActions {
        IDLE,
        ACTION,
        OVERRIDE
    }

    public void setAction(ExampleActions action) {
        currentAction = action;
    }

    public ExampleActions getAction() {
        return currentAction;
    }

    public StateMachineExample() {
        stateMachine = ExampleStateMachine.getStateMachine(ExampleStateContainer.IDLE);
    }

    @Override
    public void periodic() {
        stateMachine.fire(currentAction);
        stateMachine.getCurrentState().getState().periodic();
    }
}
