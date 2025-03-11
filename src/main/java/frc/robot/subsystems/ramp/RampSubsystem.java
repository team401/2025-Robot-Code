package frc.robot.subsystems.ramp;

import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.controls.state_machine.state.StateInterface;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.RampSubsystem.RampStates;
import frc.robot.subsystems.ramp.states.ClimbState;
import frc.robot.subsystems.ramp.states.HomingState;
import frc.robot.subsystems.ramp.states.IdleState;
import frc.robot.subsystems.ramp.states.IntakeHoldState;
import frc.robot.subsystems.ramp.states.IntakeState;
import frc.robot.subsystems.ramp.states.RampState;
import frc.robot.subsystems.ramp.states.RampState.RampTriggers;
import org.littletonrobotics.junction.Logger;

// TODO apply current to hold in position
public class RampSubsystem extends SubsystemBase {

  private RampMechanism mechanism;
  private StateMachine<RampStates, RampTriggers> stateMachine;

  public enum RampStates implements StateContainer {
    IDLE(new IdleState()),
    CLIMB(new ClimbState()),
    INTAKE(new IntakeState()),
    INTAKE_HOLD(new IntakeHoldState()),
    CLIMB_TO_INTAKE_HOMING(new HomingState()),
    HOMING(new HomingState());

    private RampState state;

    @Override
    public StateInterface getState() {
      return state;
    }

    public RampState getRampState() {
      return state;
    }

    RampStates(RampState state) {
      this.state = state;
    }
  }

  public RampSubsystem(RampMechanism rampMechanism) {
    mechanism = rampMechanism;
    setupStateMachine(mechanism);
    mechanism.setInPositionSupplier(this::isInPosition);
  }

  @Override
  public void periodic() {
    if (!DriverStation.isTestEnabled()) {
      stateMachine.periodic();
    }
    mechanism.periodic();
    Logger.recordOutput("ramp/state", stateMachine.getCurrentState());
  }

  private void setupStateMachine(RampMechanism mechanism) {
    StateMachineConfiguration<RampStates, RampTriggers> config = new StateMachineConfiguration<>();

    config
        .configure(RampStates.IDLE)
        .permit(RampTriggers.START_INTAKE, RampStates.INTAKE)
        .permit(RampTriggers.START_CLIMB, RampStates.CLIMB)
        .permit(RampTriggers.START_HOMING, RampStates.HOMING);

    config
        .configure(RampStates.INTAKE)
        .permit(RampTriggers.GOTO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.START_CLIMB, RampStates.CLIMB)
        .permit(RampTriggers.HOLD_INTAKE, RampStates.INTAKE_HOLD);

    config
        .configure(RampStates.INTAKE_HOLD)
        .permit(RampTriggers.GOTO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.START_CLIMB, RampStates.CLIMB);

    config
        .configure(RampStates.CLIMB)
        .permit(RampTriggers.GOTO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.START_INTAKE, RampStates.CLIMB_TO_INTAKE_HOMING);

    config.configure(RampStates.HOMING).permit(RampTriggers.HOMED, RampStates.IDLE);
    config
        .configure(RampStates.CLIMB_TO_INTAKE_HOMING)
        .permit(RampTriggers.HOMED, RampStates.INTAKE);

    stateMachine = new StateMachine<>(config, RampStates.HOMING);

    stateMachine.getCurrentState().getState().onEntry(null);

    RampState.setMechanism(mechanism);
    RampState.setFireTrigger(stateMachine::fire);
  }

  /** Must be called manually, does NOT run automatically */
  public void testPeriodic() {
    mechanism.testPeriodic();
  }

  public void setBrakeMode(boolean brake) {
    mechanism.setBrakeMode(brake);
  }

  public void prepareForClimb() {
    mechanism.setPosition(JsonConstants.rampConstants.climbPosition);
  }

  public void fireTrigger(RampTriggers trigger) {
    stateMachine.fire(trigger);
  }

  public boolean isInPosition() {
    return stateMachine.getCurrentState().getRampState().inPosition();
  }

  public double getPosition() {
    return mechanism.inputs.position;
  }
}
