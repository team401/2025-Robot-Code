package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;

public class ClimbState extends RampState {

  @Override
  public void periodic() {
    if (mechanism.inputs.position < JsonConstants.rampConstants.climbPosition) {
      setVoltage(JsonConstants.rampConstants.climbVoltage);
    } else {
      setVoltage(JsonConstants.rampConstants.climbHoldVoltage);
    }
    super.periodic();
  }

  public boolean inPosition() {
    return JsonConstants.rampConstants.climbPosition - mechanism.inputs.position < 0.0;
  }
}
