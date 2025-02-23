package frc.robot.subsystems.ramp;

import coppercore.parameter_tools.LoggedTunableNumber;
import frc.robot.Robot;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class RampMechanism {
  RampIO io;
  RampInputsAutoLogged inputs = new RampInputsAutoLogged();
  RampOutputsAutoLogged outputs = new RampOutputsAutoLogged();

  LoggedTunableNumber tunablePosition;
  LoggedTunableNumber tunableKP;
  LoggedTunableNumber tunableKI;
  LoggedTunableNumber tunableKD;

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
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              io.setPID(pid[0], pid[1], pid[2]);
            },
            tunableKP,
            tunableKI,
            tunableKD);
        break;
      default:
        break;
    }
  }

  public double position = 1.0;
  public boolean inPosition = false;
  public double positionRange = JsonConstants.rampConstants.positionRange;

  public RampMechanism(RampIO io) {
    this.io = io;

    tunablePosition = new LoggedTunableNumber("RampTunables/goalPos", position);
    final double defaultKP =
        Robot.isReal()
            ? JsonConstants.rampConstants.PID_TalonFX_P
            : JsonConstants.rampConstants.PID_SIM_P;
    final double defaultKI =
        Robot.isReal()
            ? JsonConstants.rampConstants.PID_TalonFX_I
            : JsonConstants.rampConstants.PID_SIM_I;
    final double defaultKD =
        Robot.isReal()
            ? JsonConstants.rampConstants.PID_TalonFX_D
            : JsonConstants.rampConstants.PID_SIM_D;
    tunableKP = new LoggedTunableNumber("RampTunables/kP", defaultKP);
    tunableKI = new LoggedTunableNumber("RampTunables/kI", defaultKI);
    tunableKD = new LoggedTunableNumber("RampTunables/kD", defaultKD);
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
