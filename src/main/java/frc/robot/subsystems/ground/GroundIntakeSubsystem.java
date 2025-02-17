package frc.robot.subsystems.ground;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import frc.robot.subsystems.ground.states.IdleState;
import frc.robot.subsystems.ground.states.IntakeState;
import frc.robot.subsystems.ground.states.ReverseState;
import frc.robot.subsystems.ground.states.WarmupState;

public class GroundIntakeSubsystem {

  private static GroundIntakeSubsystem instance;

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
    WARMUP(new WarmupState(instance)), // waiting for system to be ready for climb
    INTAKE(new IntakeState(instance)), // searching for target
    REVERSE(new ReverseState(instance)), // moving arm to hang
    OVERRIDE(new PeriodicStateInterface() {});

    private final PeriodicStateInterface state;

    GroundIntakeState(PeriodicStateInterface state) {
      this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
      return state;
    }
  }
}
