package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.StateMachine;

public class StateMachineExample extends SubsystemBase {

   static class IdleState implements PeriodicStateInterface {
      @Override
      public void periodic(){
         //Called in subsystem periodic if statemachine in this state.
      }
   };
   static class ActionState implements PeriodicStateInterface {
      @Override
      public void periodic(){
         //Called in subsystem periodic if statemachine in this state.
      }
   };
   static class OverrideState implements PeriodicStateInterface {
      @Override
      public void periodic(){
         //Called in subsystem periodic if statemachine in this state.
      }
   };

   public static enum ExampleStateContainer implements StateContainer {
      IDLE(new IdleState()),
      ACTION(new ActionState()),
      OVERRIDE(new OverrideState()),

      private final PeriodicStateInterface state;

      ExampleStateContainer(PeriodicStateInterface state) {
         this.state = state;
      }

      @Override
      public PeriodicStateInterface getState() {
         return state;
      }
   }

   public static enum ExampleActions {
      IDLE,
      ACTION,
      OVERRIDE
   }

   private StateMachine<ExampleStateContainer, ExampleActions> stateMachine;

   private ExampleActions currentAction = ExampleActions.IDLE;

   public void setAction(ExampleActions action){
      currentAction = action;
   }

   public ExampleActions getAction(){
      return currentAction;
   }

   public static void setupStateMachine(){
      StateMachineConfiguration<ExampleStateContainer, ExampleActions> stateMachineConfig = new StateMachineConfiguration<>();

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


      stateMachine = new StateMachine<>(stateMachineConfig, ExampleStateContainer.IDLE);
   }
   
   public StateMachineExample(){
      setupStateMachine();
   }
   
   @Override
   public void periodic() {
      stateMachine.fire(currentAction);
      stateMachine.getCurrentState().getState().periodic();
   }
   
}