package frc.robot.subsystems.ramp.states;

import coppercore.controls.state_machine.transition.Transition;
import frc.robot.constants.JsonConstants;

public class IntakeHoldState extends RampState {

  @Override
  public void periodic() {
    setVoltage(JsonConstants.rampConstants.intakeVoltage);
    super.periodic();
  }

  @Override
  public void onExit(Transition transition) {
    mechanism.setHome();
  }
}
