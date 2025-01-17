package frc.robot.subsystems.stateMachineExample.state_machine;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import frc.robot.subsystems.stateMachineExample.StateMachineExample.ExampleActions;

public class ExampleStateMachine {

    private static StateMachineConfiguration<ExampleStateContainer, ExampleActions>
            getStateMachineConfiguration() {
        StateMachineConfiguration<ExampleStateContainer, ExampleActions> stateMachineConfig =
                new StateMachineConfiguration<>();

        stateMachineConfig
                .configure(ExampleStateContainer.IDLE)
                .permit(ExampleActions.ACTION, ExampleStateContainer.ACTION)
                .permit(ExampleActions.OVERRIDE, ExampleStateContainer.OVERRIDE);

        stateMachineConfig
                .configure(ExampleStateContainer.ACTION)
                .permit(ExampleActions.IDLE, ExampleStateContainer.IDLE)
                .permit(ExampleActions.OVERRIDE, ExampleStateContainer.OVERRIDE);

        stateMachineConfig
                .configure(ExampleStateContainer.OVERRIDE)
                .permit(ExampleActions.ACTION, ExampleStateContainer.ACTION)
                .permit(ExampleActions.IDLE, ExampleStateContainer.IDLE);
        return stateMachineConfig;
    }

    public static StateMachine<ExampleStateContainer, ExampleActions> getStateMachine(
            ExampleStateContainer initialState) {
        return new StateMachine<>(getStateMachineConfiguration(), initialState);
    }
}
