package frc.robot.subsystems.ground;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import frc.robot.subsystems.ground.states.IdleState;

public class GroundSubsystem {

  private GroundSubsystem instance = this;

  GroundIntakeIO io;
  GroundIntakeInputsAutoLogged inputs = new GroundIntakeInputsAutoLogged();
  GroundIntakeOutputsAutoLogged outputs = new GroundIntakeOutputsAutoLogged();

  public enum GroundIntakeAction {
    NONE, // do nothing
    INTAKE,
    REVERSE,
    CANCEL,
    OVERRIDE
  }

   private enum GroundIntakeState implements StateContainer {
    IDLE(new IdleState(instance)), // do nothing
    WAITING(new WaitingState(instance)), // waiting for system to be ready for climb
    SEARCHING(new SearchingState(instance)), // searching for target
    LIFTING(new LiftingState(instance)), // moving arm to hang
    OVERRIDE(new PeriodicStateInterface() {});

    private final PeriodicStateInterface state;

    ClimbState(PeriodicStateInterface state) {
      this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
      return state;
    }
  }

}
