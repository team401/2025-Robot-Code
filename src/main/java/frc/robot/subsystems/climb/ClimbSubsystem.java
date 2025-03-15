package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Radians;
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

  LoggedTunableNumber climbTuningSetpointRadians;
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
        .permit(ClimbAction.CLIMB, ClimbState.LIFTING) // , () -> inputs.lockedToCage)
        .permit(ClimbAction.CANCEL, ClimbState.IDLE);

    climbMachineConfiguration
        .configure(ClimbState.LIFTING)
        .permit(ClimbAction.CANCEL, ClimbState.IDLE);

    climbMachineConfiguration
        .configure(ClimbState.OVERRIDE)
        .permit(ClimbAction.CANCEL, ClimbState.IDLE);

    climbMachine =
        new StateMachine<ClimbState, ClimbAction>(climbMachineConfiguration, ClimbState.IDLE);

    climbkP =
        new LoggedTunableNumber("ClimbTunables/climbkP", ClimbConstants.synced.getObject().climbkP);
    climbkI =
        new LoggedTunableNumber("ClimbTunables/climbkI", ClimbConstants.synced.getObject().climbkI);
    climbkD =
        new LoggedTunableNumber("ClimbTunables/climbkD", ClimbConstants.synced.getObject().climbkD);

    climbkS =
        new LoggedTunableNumber("ClimbTunables/climbkS", ClimbConstants.synced.getObject().climbkS);
    climbkV =
        new LoggedTunableNumber("ClimbTunables/climbkV", ClimbConstants.synced.getObject().climbkV);
    climbkA =
        new LoggedTunableNumber("ClimbTunables/climbkA", ClimbConstants.synced.getObject().climbkA);
    climbkG =
        new LoggedTunableNumber("ClimbTunables/climbkG", ClimbConstants.synced.getObject().climbkG);

    climbTuningSetpointRadians =
        new LoggedTunableNumber("ClimbTunables/climbTuningSetpointRadians", 0.0);
    climbTuningOverrideVolts =
        new LoggedTunableNumber("ClimbTunables/climbTuningOverrideVolts", 0.0);
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

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
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
        fireTrigger(ClimbAction.OVERRIDE);
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

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (setpoint) -> {
              setGoalAngle(Radians.of(setpoint[0]));
            },
            climbTuningSetpointRadians);
        break;
      default:
        break;
    }
  }

  public void setPID(double newP, double newI, double newD) {
    io.setPID(newP, newI, newD);
  }
}
