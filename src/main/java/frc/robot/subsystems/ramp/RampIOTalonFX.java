package frc.robot.subsystems.ramp;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.constants.JsonConstants;

// TODO make pid constants
public class RampIOTalonFX implements RampIO {

  private final PIDController controller =
      new PIDController(
          JsonConstants.rampConstants.PID_TalonFX_P,
          JsonConstants.rampConstants.PID_TalonFX_I,
          JsonConstants.rampConstants.PID_TalonFX_D);

  private TalonFX talon;

  public RampIOTalonFX() {
    talon = new TalonFX(JsonConstants.rampConstants.motorId);
  }

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  @Override
  public void updateInputs(RampInputs inputs) {
    inputs.position = Rotations.of(talon.getPosition().getValueAsDouble()).in(Radians);
  }

  @Override
  public void updateOutputs(RampInputs inputs, RampOutputs outputs) {
    double volts = inputs.controlValue;
    if (inputs.positionControl){
      controller.setSetpoint(inputs.controlValue);
      volts = controller.calculate(inputs.position);
    }
    volts = Math.min(Math.max(volts, -12.0), 12.0);
    outputs.appliedVolts = volts;
    talon.setVoltage(volts);
  }
}
