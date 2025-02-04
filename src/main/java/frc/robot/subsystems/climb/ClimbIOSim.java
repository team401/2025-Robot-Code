package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import java.util.function.BooleanSupplier;

public class ClimbIOSim implements ClimbIO {

  private SingleJointedArmSim climb =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(2),
          75,
          SingleJointedArmSim.estimateMOI(0.5, 0.5),
          0.5,
          0,
          2 * Math.PI,
          false,
          0,
          0.005,
          0);

  private final PIDController angleController = new PIDController(5, 0, 0);

  private BooleanSupplier lockedToCage = () -> true;

  private MutAngle goalAngle = Radians.mutable(0);
  private MutVoltage overrideVoltage = Volts.mutable(0.0);

  private boolean override = false;

  public ClimbIOSim() {}

  @Override
  public void updateInputs(ClimbInputs inputs) {

    inputs.lockedToCage = this.lockedToCage.getAsBoolean();
    inputs.goalAngle.mut_replace(goalAngle);
    inputs.motorAngle.mut_replace(Radians.of(climb.getAngleRads()));
  }

  @Override
  public void applyOutputs(ClimbOutputs outputs) {
    Voltage appliedVolts;
    if (override) {
      appliedVolts = overrideVoltage;
    } else {
      appliedVolts =
          Volts.of(angleController.calculate(climb.getAngleRads(), goalAngle.in(Radians)));
    }

    outputs.appliedVoltage.mut_replace(appliedVolts);
    climb.setInputVoltage(appliedVolts.in(Volts));
    climb.update(0.02);
  }

  @Override
  public void setGoalAngle(Angle angle) {
    override = false;
    goalAngle.mut_replace(angle);
  }

  @Override
  public void setOverrideVoltage(Voltage voltage) {
    override = true;
    overrideVoltage.mut_replace(voltage);
  }

  @Override
  public void setPID(double p, double i, double d) {
    angleController.setPID(p, i, d);
  }
}
