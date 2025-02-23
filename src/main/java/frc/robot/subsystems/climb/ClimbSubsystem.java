package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.units.measure.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.TestModeManager;
import frc.robot.constants.ClimbConstants;
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

  LoggedTunableNumber climbkP;
  LoggedTunableNumber climbkI;
  LoggedTunableNumber climbkD;

  LoggedTunableNumber climbkS;
  LoggedTunableNumber climbkV;
  LoggedTunableNumber climbkA;
  LoggedTunableNumber climbkG;

  LoggedTunableNumber climbTuningSetpointMeters;
  LoggedTunableNumber climbTuningOverrideVolts;

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

    climbkP =
        new LoggedTunableNumber("climbTunables/climbkP", ClimbConstants.synced.getObject().climbkP);
    climbkI =
        new LoggedTunableNumber("climbTunables/climbkI", ClimbConstants.synced.getObject().climbkI);
    climbkD =
        new LoggedTunableNumber("climbTunables/climbkD", ClimbConstants.synced.getObject().climbkD);

    climbkS =
        new LoggedTunableNumber("climbTunables/climbkS", ClimbConstants.synced.getObject().climbkS);
    climbkV =
        new LoggedTunableNumber("climbTunables/climbkV", ClimbConstants.synced.getObject().climbkV);
    climbkA =
        new LoggedTunableNumber("climbTunables/climbkA", ClimbConstants.synced.getObject().climbkA);
    climbkG =
        new LoggedTunableNumber("climbTunables/climbkG", ClimbConstants.synced.getObject().climbkG);

    climbTuningSetpointMeters =
        new LoggedTunableNumber("climbTunables/climbTuningSetpointMeters", 0.0);
    climbTuningOverrideVolts =
        new LoggedTunableNumber("climbTunables/climbTuningOverrideVolts", 0.0);
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

  public void setRampClear(BooleanSupplier rClear) {
    rampClear = rClear;
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

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case ClimbTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            climbkP,
            climbkI,
            climbkD);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (ff) -> {
              io.setFF(ff[0], ff[1], ff[2], ff[3]);
            },
            climbkS,
            climbkV,
            climbkA,
            climbkG);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              io.setOverrideVoltage(Volts.of(setpoint[0]));
            },
            climbTuningOverrideVolts);

      case SetpointTuning:
        // Allow setpointing the climb in climbTuning and SetpointTuning modes
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              setGoalAngle(Degrees.of(setpoint[0]));
            },
            climbTuningSetpointMeters);

        break;
      default:
        break;
    }
  }
}
