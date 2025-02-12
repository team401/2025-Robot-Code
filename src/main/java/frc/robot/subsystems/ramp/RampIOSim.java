package frc.robot.subsystems.ramp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.JsonConstants;

// TODO make pid constants
public class RampIOSim implements RampIO {

  private final PIDController controller =
      new PIDController(
          JsonConstants.rampConstants.PID_SIM_P,
          JsonConstants.rampConstants.PID_SIM_I,
          JsonConstants.rampConstants.PID_SIM_D);

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), 25, 10, 24.938, 0, 2 * Math.PI, true, Math.PI);

  @Override
  public void updateInputs(RampInputs inputs) {
    sim.update(0.02);
    inputs.position = sim.getAngleRads();
  }

  @Override
  public void updateOutputs(RampInputs inputs, RampOutputs outputs) {
    controller.setSetpoint(outputs.targetPosition);
    double volts = controller.calculate(inputs.position);
    volts = Math.min(Math.max(volts, -12.0), 12.0);
    outputs.appliedVolts = volts;
    sim.setInputVoltage(volts);
  }
}
