package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ClimbConstants;
import java.util.function.BooleanSupplier;

public class ClimbIOTalonFX implements ClimbIO {

  TalonFX leadMotor;
  TalonFX followerMotor;

  private final PIDController angleController =
      new PIDController(
          ClimbConstants.synced.getObject().climbP,
          ClimbConstants.synced.getObject().climbI,
          ClimbConstants.synced.getObject().climbD);

  // TODO: replace when sensors become available
  private BooleanSupplier lockedToCage = () -> true;

  private MutAngle goalAngle = Radians.mutable(0);
  private MutVoltage overrideVoltage = Volts.mutable(0.0);

  private boolean override = false;

  private MotionMagicExpoDutyCycle calculator;

  public ClimbIOTalonFX() {
    leadMotor = new TalonFX(ClimbConstants.synced.getObject().leadClimbMotorId);
    followerMotor = new TalonFX(ClimbConstants.synced.getObject().followerClimbMotorId);

    followerMotor.setControl(
        new Follower(
            leadMotor.getDeviceID(), ClimbConstants.synced.getObject().invertFollowerClimbMotor));

    // TODO: set lockedToCage when ramp becomes available

  }

  @Override
  public void updateInputs(ClimbInputs inputs) {

    inputs.lockedToCage = this.lockedToCage.getAsBoolean();
    inputs.goalAngle.mut_replace(goalAngle);
    inputs.motorAngle.mut_replace(leadMotor.getPosition().getValue());
  }

  @Override
  public void applyOutputs(ClimbOutputs outputs) {

    calculator.withPosition(leadMotor.getPosition().getValue().in(Radians));

    Voltage appliedVolts;
    if (override) {
      appliedVolts = overrideVoltage;
    } else {
      appliedVolts =
          Volts.of(
              angleController.calculate(
                  leadMotor.getPosition().getValue().in(Radians), goalAngle.in(Radians)));
    }

    outputs.appliedVoltage.mut_replace(appliedVolts);
    leadMotor.setVoltage(appliedVolts.in(Volts));
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
