package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ramp.states.RampState.RampTriggers;

public class IdleState extends RampState {

  @Override
  public void periodic() {
    if (mechanism.inputs.position >= JsonConstants.rampConstants.autoHomePositionHigh
        || mechanism.inputs.position <= JsonConstants.rampConstants.autoHomePositionLow) {
      fireTrigger.accept(RampTriggers.START_HOMING);
    } else {
      setVoltage(0.0);
    }
    super.periodic();
  }
}
