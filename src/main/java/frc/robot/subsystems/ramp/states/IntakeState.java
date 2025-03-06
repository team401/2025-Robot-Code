package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;

public class IntakeState extends RampState {

  @Override
  public void periodic() {
    setVoltage(JsonConstants.rampConstants.intakeVoltage);
    if (inPosition()) {
      fireTrigger.accept(RampTriggers.HOLD_INTAKE);
    }
    super.periodic();
  }
}
