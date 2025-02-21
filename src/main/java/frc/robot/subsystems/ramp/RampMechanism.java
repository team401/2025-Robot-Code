package frc.robot.subsystems.ramp;

import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class RampMechanism {
  RampIO io;
  public RampInputsAutoLogged inputs = new RampInputsAutoLogged();
  RampOutputsAutoLogged outputs = new RampOutputsAutoLogged();

  public double position = 1.0;
  public boolean inPosition = false;
  public double positionRange = JsonConstants.rampConstants.positionRange;

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

  public void setVoltage(double voltage) {
    this.voltage = voltage;
    positionControl = false;
  }
}
