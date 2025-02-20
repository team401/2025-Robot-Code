package frc.robot.subsystems.climb;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climb.states.IdleState;
import frc.robot.subsystems.climb.states.LiftingState;
import frc.robot.subsystems.climb.states.SearchingState;
import frc.robot.subsystems.climb.states.WaitingState;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimbSubsystem extends SubsystemBase {

  private static ClimbSubsystem instance;

  private ClimbIO io;
  private ClimbInputsAutoLogged inputs = new ClimbInputsAutoLogged();
  private ClimbOutputsAutoLogged outputs = new ClimbOutputsAutoLogged();

  // TODO: replace when ramp becomes a thing
  private BooleanSupplier rampClear = () -> true;

  public enum ClimbAction {
    NONE, // do nothing
    CLIMB,
    CANCEL,
    OVERRIDE
  }

  ClimbAction currentAction = ClimbAction.NONE;

  private enum ClimbState implements StateContainer {
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

  public StateMachineConfiguration<ClimbState, ClimbAction> climbMachineConfiguration;
  public StateMachine<ClimbState, ClimbAction> climbMachine;

  public ClimbSubsystem(ClimbIO io) {
    instance = this;
    this.io = io;

    climbMachineConfiguration = new StateMachineConfiguration<>();

    climbMachineConfiguration
        .configure(ClimbState.IDLE)
        .permit(ClimbAction.CLIMB, ClimbState.WAITING)
        .permit(ClimbAction.OVERRIDE, ClimbState.OVERRIDE);

    climbMachineConfiguration
        .configure(ClimbState.WAITING)
        .permitIf(ClimbAction.CLIMB, ClimbState.SEARCHING, rampClear)
        .permit(ClimbAction.CANCEL, ClimbState.IDLE);

    climbMachineConfiguration
        .configure(ClimbState.SEARCHING)
        .permitIf(ClimbAction.CLIMB, ClimbState.LIFTING, () -> inputs.lockedToCage)
        .permit(ClimbAction.CANCEL, ClimbState.IDLE);

    climbMachineConfiguration
        .configure(ClimbState.LIFTING)
        .permit(ClimbAction.CANCEL, ClimbState.IDLE);

    climbMachine =
        new StateMachine<ClimbState, ClimbAction>(climbMachineConfiguration, ClimbState.IDLE);
  }

  public void fireTrigger(ClimbAction action) {
    currentAction = action;
    climbMachine.fire(action);
  }

  public void setGoalAngle(Angle angle) {
    io.setGoalAngle(angle);
  }

  public boolean getRampClear() {
    return rampClear.getAsBoolean();
  }

  public boolean getLockedToCage() {
    return inputs.lockedToCage;
  }

  public Angle getRotation() {
    return inputs.motorAngle;
  }

  @Override
  public void periodic() {

    climbMachine.periodic();

    io.updateInputs(inputs);
    io.applyOutputs(outputs);

    Logger.processInputs("climb/inputs", inputs);
    Logger.processInputs("climb/outputs", outputs);
    Logger.recordOutput("climb/rampClear", rampClear);
    Logger.recordOutput("climb/State", climbMachine.getCurrentState());
    Logger.recordOutput("climb/Action", currentAction);
  }
}
