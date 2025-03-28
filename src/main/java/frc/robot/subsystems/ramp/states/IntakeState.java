package frc.robot.subsystems.ramp.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.constants.JsonConstants;

public class IntakeState extends RampState {

  public boolean hasReachedGoal = false;

  @Override
  public void onEntry(Transition transition) {
    hasReachedGoal = false;
  }

  @Override
  public void periodic() {
    if (!hasReachedGoal) {
      setVoltage(JsonConstants.rampConstants.intakeVoltage);
      hasReachedGoal = inPosition();
    } else {
      setVoltage(JsonConstants.rampConstants.intakeHoldVoltage);
    }
    super.periodic();
  }

  @Override
  public void onExit(Transition transition) {
    if (hasReachedGoal) {
      mechanism.setHome();
    }
  }
}
