package frc.robot.subsystems.ramp;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

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

  private final double gearing = 25;
  private final double jKgMetersSquared = 0.01906254;
  private final double armLengthMeters = Inches.of(24.938).in(Meters);
  private final double minAngleRads = 0;
  private final double maxAngleRads = Math.PI;
  private final double startingRads = 0.5 * Math.PI;
  private final boolean simulateGravity = true;

  public double angle_offset = 0.0;

  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          gearing,
          jKgMetersSquared,
          armLengthMeters,
          minAngleRads,
          maxAngleRads,
          simulateGravity,
          startingRads);

  @Override
  public void setPID(double kP, double kI, double kD) {
    controller.setP(kP);
    controller.setI(kI);
    controller.setD(kD);
  }

  @Override
  public void addOffset(double offset) {
    angle_offset += offset;
  }

  @Override
  public void updateInputs(RampInputs inputs) {
    sim.update(0.02);
    inputs.position = sim.getAngleRads() + angle_offset;
  }

  @Override
  public void updateOutputs(RampInputs inputs, RampOutputs outputs) {
    outputs.velocity = sim.getVelocityRadPerSec();
    double volts = inputs.controlValue;
    if (inputs.positionControl) {
      controller.setSetpoint(inputs.controlValue);
      volts = controller.calculate(inputs.position);
    }
    volts = Math.min(Math.max(volts, -12.0), 12.0);
    outputs.appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  public void setBrakeMode(boolean brake) {
    return;
  }
}
