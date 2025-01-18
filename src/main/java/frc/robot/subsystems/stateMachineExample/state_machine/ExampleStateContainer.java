package frc.robot.subsystems.stateMachineExample.state_machine;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import frc.robot.subsystems.stateMachineExample.state_machine.states.ActionState;
import frc.robot.subsystems.stateMachineExample.state_machine.states.IdleState;
import frc.robot.subsystems.stateMachineExample.state_machine.states.OverrideState;

public enum ExampleStateContainer implements StateContainer {
    IDLE(new IdleState()),
    ACTION(new ActionState()),
    OVERRIDE(new OverrideState());

    private final PeriodicStateInterface state;

    ExampleStateContainer(PeriodicStateInterface state) {
        this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
        return state;
    }
}
