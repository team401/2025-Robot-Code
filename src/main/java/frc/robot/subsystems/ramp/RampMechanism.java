package frc.robot.subsystems.ramp;

import org.littletonrobotics.junction.Logger;

import coppercore.parameter_tools.LoggedTunableNumber;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;

public class RampMechanism {
  RampIO io;
  RampInputsAutoLogged inputs = new RampInputsAutoLogged();
  RampOutputsAutoLogged outputs = new RampOutputsAutoLogged();

  

  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case RampTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pos) -> {
              position = pos[0];
              outputs.targetPosition = position;
            },
            tunablePosition);
        break;
      default:
        break;
    }
  }

  public double position = 1.0;
  public boolean inPosition = false;
  public double positionRange = JsonConstants.rampConstants.positionRange;
  LoggedTunableNumber tunablePosition =
  new LoggedTunableNumber(
      "RampTunables/setpoint", position);

  public RampMechanism(RampIO io) {
    this.io = io;
  }

  private void updateInPosition() {
    inPosition = Math.abs(position - inputs.position) <= positionRange;
  }

  public void periodic() {
    io.updateInputs(inputs);
    updateInPosition();
    outputs.targetPosition = position;
    inputs.inPosition = inPosition;
    io.updateOutputs(inputs, outputs);
    Logger.processInputs("ramp/inputs", inputs);
    Logger.processInputs("ramp/outputs", outputs);
  }

  public void setPosition(double position) {
    this.position = position;
    updateInPosition();
  }

  public boolean inPosition() {
    return inPosition;
  }

  public boolean inTransition() {
    return !inPosition;
  }
}
