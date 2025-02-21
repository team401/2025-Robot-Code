package frc.robot.subsystems.ramp;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.controls.state_machine.state.StateInterface;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.states.IntakeState;

public class RampSubsystem extends SubsystemBase {

  private RampMechanism mechanism;
  private StateMachine<RampStates, RampTriggers> stateMachine;

  public static enum RampTriggers {
    GOTO_IDLE,
    START_INTAKE,
    HOLD_INTAKE,
    START_CLIMB
  }

  public static enum RampStates implements StateContainer {
    IDLE(new IdleState()),
    CLIMB(new ClimbState()),
    INTAKE(new IntakeState()),
    INTAKE_HOLD(new IntakeHoldState());

    private RampState state;

    @Override
    public StateInterface getState(){
        return state;
    }

    public RampState getRampState(){
        return state;
    }

    RampStates(RampState state){
      this.state = state;
    }
  }


  public RampSubsystem(RampMechanism rampMechanism) {
     mechanism = rampMechanism;
     stateMachine = setupStateMachine(mechanism);
  }

  public void fireTrigger(RampTriggers trigger){
     stateMachine.fire(trigger);
  }
  
  @Override
  public void periodic() {
     stateMachine.periodic();
     mechanism.periodic();
  }

  public boolean isInPosition() {
     return ((RampState) stateMachine.getState().getState()).inPosition();
  }

  public double getPosition() {
     return mechanism.inputs.position;
  }

  private StateMachine setupStateMachine(RampMechanism mechanism){
     StateMachineConfiguration config = new StateMachineConfiguration();
  
     config
        .configure(RampStates.IDLE)
        .permit(RampTriggers.START_INTAKE, RampStates.INTAKE)
        .permit(RampTriggers.START_CLIMB, RampStates.CLIMB);
  
     config
        .configure(RampStates.INTAKE)
        .permit(RampTriggers.GOTO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.START_CLIMB, RampStates.CLIMB)
        .permit(RampTriggers.HOLD_INTAKE, RampStates.INTAKE_HOLD);
  
     config
        .configure(RampStates.INTAKE_HOLD)
        .permit(RampTriggers.GOTO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.START_CLIMB, RampStates.CLIMB)
  
     config
        .configure(RampStates.CLIMB)
        .permit(RampTriggers.GOTO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.START_INTAKE, RampStates.INTAKE);
  
     StateMachine stateMachine = new StateMachine(config, RampStates.IDLE);
  
     RampState.setMechanism(mechanism);
     RampState.setFireTrigger(stateMachine::fire);
     
     return stateMachine;
  }
  
}