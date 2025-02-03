package frc.robot.subsystems.ramp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

// TODO make pid constants
public class RampIOSim implements RampIO {

  private PIDTuner tuner = new PIDTuner(PIDTuner.ControlType.PID);

  // private PIDController controller = new PIDController(7.5, 0.001, 0.0);
  // private PIDController controller =
  // new PIDController(78.07672351482793, 10.049834881962159, 2.5124587204905398);
  private PIDController controller = new PIDController(0, 0, 0);

  private SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1), 25, 10, 24.938, 0, 2 * Math.PI, true, Math.PI);

  @Override
  public void updateInputs(RampInputs inputs) {
    sim.update(0.02);
    inputs.position = sim.getAngleRads();
  }

  private int count = 0;
  private double time;

  @Override
  public void updateOutputs(RampInputs inputs, RampOutputs outputs) {
    time += 0.02;
    controller.setSetpoint(outputs.targetPosition);
    tuner.recordData(controller.getError(), 0.02);
    tuner.tune();
    PIDTuner.PID pid = tuner.getPIDToUse();
    controller.setP(pid.P());
    controller.setI(pid.I());
    controller.setD(pid.D());
    Logger.recordOutput("Error", controller.getError());
    Logger.recordOutput("Time", time);
    Logger.recordOutput("Finished", tuner.hasFinished());
    Logger.recordOutput("Kp", pid.P());
    Logger.recordOutput("Ki", pid.I());
    Logger.recordOutput("Kd", pid.D());

    double volts = controller.calculate(inputs.position);
    outputs.appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  private void setMotorVolts() {}
}
