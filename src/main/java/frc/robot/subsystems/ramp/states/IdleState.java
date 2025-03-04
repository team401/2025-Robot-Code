package frc.robot.subsystems.ramp.states;

import frc.robot.constants.JsonConstants;

public class IdleState extends RampState {

  @Override
  public void periodic() {
    setPosition(JsonConstants.rampConstants.idlePosition);
    super.periodic();
  }
}
