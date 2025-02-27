package frc.robot.subsystems.ramp;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.units.measure.AngularVelocity;
import frc.robot.Robot;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class RampMechanism {
  RampIO io;
  public RampInputsAutoLogged inputs = new RampInputsAutoLogged();
  RampOutputsAutoLogged outputs = new RampOutputsAutoLogged();

  private boolean positionControl = false;
  private double controlValue = 0.0;

  public BooleanSupplier inPositionBooleanSupplier;

  LoggedTunableNumber tunablePosition;
  LoggedTunableNumber tunableVoltage;
  LoggedTunableNumber tunableKP;
  LoggedTunableNumber tunableKI;
  LoggedTunableNumber tunableKD;

  public void setInPositionSupplier(BooleanSupplier inPositionSupplier) {
    inPositionBooleanSupplier = inPositionSupplier;
  }

  public RampMechanism(RampIO io) {
    this.io = io;
    tunablePosition = new LoggedTunableNumber("RampTunables/goalPos", controlValue);
    tunableVoltage = new LoggedTunableNumber("RampTunables/voltage", controlValue);
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

  public void periodic() {

    inputs.positionControl = positionControl;
    inputs.controlValue = controlValue;

    io.updateInputs(inputs);
    io.updateOutputs(inputs, outputs);

    Logger.recordOutput("ramp/inPosition", inPositionBooleanSupplier.getAsBoolean());
    Logger.processInputs("ramp/inputs", inputs);
    Logger.processInputs("ramp/outputs", outputs);
  }

  public void setPosition(double position) {
    controlValue = position;
    positionControl = true;
  }

  public void setVoltage(double voltage) {
    controlValue = voltage;
    positionControl = false;
  }

  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case RampTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pos) -> {
              setPosition(pos[0]);
            },
            tunablePosition);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (volt) -> {
              setVoltage(volt[0]);
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

  public void setHome() {
    io.addOffset(-inputs.position);
  }

  public AngularVelocity getVelocity() {
    return RadiansPerSecond.of(outputs.velocity);
  }
}
