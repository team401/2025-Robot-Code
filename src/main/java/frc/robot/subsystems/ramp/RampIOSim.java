package frc.robot.subsystems.ramp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

// TODO make pid constants
public class RampIOSim implements RampIO {

    private PIDController controller = new PIDController(5.0, 0.05, 0.0);

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
        setMotorVolts();
        controller.setSetpoint(outputs.targetPosition);
        double volts = controller.calculate(inputs.position);
        outputs.appliedVolts = volts;
        sim.setInputVoltage(volts);
    }

    private void setMotorVolts() {}
}
