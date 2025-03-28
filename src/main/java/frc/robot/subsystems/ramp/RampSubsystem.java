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
import frc.robot.subsystems.ramp.states.ExtendingState;
import frc.robot.subsystems.ramp.states.HomingState;
import frc.robot.subsystems.ramp.states.IdleState;
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
    CLIMB_POSITION(new ClimbState()),
    INTAKE_POSITION(new IntakeState()),
    EXTENDING(new ExtendingState()),
    CLIMB_TO_INTAKE_HOMING(new HomingState()),
    STARTUP_HOMING(new HomingState()),
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
        .permit(RampTriggers.INTAKE, RampStates.INTAKE_POSITION)
        .permit(RampTriggers.CLIMB, RampStates.CLIMB_POSITION)
        .permit(RampTriggers.HOME, RampStates.HOMING);

    config
        .configure(RampStates.INTAKE_POSITION)
        .permit(RampTriggers.RETURN_TO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.CLIMB, RampStates.CLIMB_POSITION)
        .permit(RampTriggers.HOME, RampStates.HOMING);

    config
        .configure(RampStates.CLIMB_POSITION)
        .permit(RampTriggers.RETURN_TO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.INTAKE, RampStates.CLIMB_TO_INTAKE_HOMING)
        .permit(RampTriggers.HOME, RampStates.HOMING);

    config.configure(RampStates.HOMING).permit(RampTriggers.HOMING_FINISHED, RampStates.IDLE);

    config
        .configure(RampStates.STARTUP_HOMING)
        .permit(RampTriggers.HOMING_FINISHED, RampStates.EXTENDING);

    config.configure(RampStates.EXTENDING).permit(RampTriggers.HOME, RampStates.HOMING);

    config
        .configure(RampStates.CLIMB_TO_INTAKE_HOMING)
        .permit(RampTriggers.HOMING_FINISHED, RampStates.INTAKE_POSITION)
        .permit(RampTriggers.RETURN_TO_IDLE, RampStates.IDLE)
        .permit(RampTriggers.HOME, RampStates.HOMING);

    stateMachine = new StateMachine<>(config, RampStates.STARTUP_HOMING);

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
