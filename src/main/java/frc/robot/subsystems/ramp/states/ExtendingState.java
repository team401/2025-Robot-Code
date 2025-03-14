package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.states.RampState.RampTriggers;

public class ExtendingState extends RampState {

  @Override
  public void periodic() {
    if (mechanism.inputs.position < JsonConstants.rampConstants.extendPosition) {
      setVoltage(JsonConstants.rampConstants.extendingVoltage);
    } else {
      fireTrigger.accept(RampTriggers.HOME);
    }
    super.periodic();
  }

  @Override
  public boolean inPosition() {
    return false;
  }
}
