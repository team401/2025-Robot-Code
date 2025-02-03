package frc.robot.subsystems.ramp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

// TODO make pid constants
public class RampIOSim implements RampIO {

  private PidAutoTuner tuner = new PidAutoTuner();

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
    tuner.addData(controller.getError(), 0.02);
    if (count < 10) {
      if (time % 5.0 < 0.02) {
        tuner.update();
        Logger.recordOutput("Kp", tuner.getP());
        Logger.recordOutput("Ki", tuner.getI());
        Logger.recordOutput("Kd", tuner.getD());
        controller.setP(tuner.getP());
        controller.setI(tuner.getI());
        controller.setD(tuner.getD());
        controller.reset();
        // tuner.reset();
        // sim.setState(Math.PI, 0);
      }
      if (time > 15.0) {
        tuner.update();
        Logger.recordOutput("Kp", tuner.getP());
        Logger.recordOutput("Ki", tuner.getI());
        Logger.recordOutput("Kd", tuner.getD());
        controller.setP(tuner.getP());
        controller.setI(tuner.getI());
        controller.setD(tuner.getD());
        controller.reset();
        tuner.reset();
        sim.setState(Math.PI, 0);
        time = 0;
        count++;
      }
    }
    Logger.recordOutput("Error", controller.getError());
    Logger.recordOutput("Time", time);

    double volts = controller.calculate(inputs.position);
    outputs.appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  private void setMotorVolts() {}
}
