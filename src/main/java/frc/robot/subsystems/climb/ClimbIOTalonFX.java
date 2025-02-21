package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ClimbConstants;
import java.util.function.BooleanSupplier;

public class ClimbIOTalonFX implements ClimbIO {

  TalonFX leadMotor;
  TalonFX followerMotor;

  TalonFXConfiguration talonFXConfigs;

  // TODO: replace when sensors become available
  private BooleanSupplier lockedToCage = () -> true;

  private MutAngle goalAngle = Radians.mutable(0);
  private MutVoltage overrideVoltage = Volts.mutable(0.0);

  private boolean override = false;

  private MotionMagicVoltage calculator =
      new MotionMagicVoltage(ClimbConstants.synced.getObject().restingAngle);

  public ClimbIOTalonFX() {
    leadMotor = new TalonFX(ClimbConstants.synced.getObject().leadClimbMotorId);
    followerMotor = new TalonFX(ClimbConstants.synced.getObject().followerClimbMotorId);

    followerMotor.setControl(
        new Follower(
            leadMotor.getDeviceID(), ClimbConstants.synced.getObject().invertFollowerClimbMotor));

    talonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(ClimbConstants.synced.getObject().climbCurrentLimit))
            .withSlot0(
                new Slot0Configs()
                    .withKS(ClimbConstants.synced.getObject().climbkS)
                    .withKV(ClimbConstants.synced.getObject().climbkV)
                    .withKA(ClimbConstants.synced.getObject().climbkA)
                    .withKG(ClimbConstants.synced.getObject().climbkG)
                    .withKP(ClimbConstants.synced.getObject().climbkP)
                    .withKI(ClimbConstants.synced.getObject().climbkI)
                    .withKD(ClimbConstants.synced.getObject().climbkD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicAcceleration(5)
                    .withMotionMagicCruiseVelocity(5));

    leadMotor.getConfigurator().apply(talonFXConfigs);
    followerMotor.getConfigurator().apply(talonFXConfigs);

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

    calculator.withPosition(goalAngle);

    if (override) {
      leadMotor.setVoltage(0);
    } else {
      leadMotor.setControl(calculator);
    }

    outputs.appliedVoltage.mut_replace(leadMotor.getMotorVoltage().getValue());
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
  public void setBrakeMode(boolean brake) {
    if (brake) {
      leadMotor
          .getConfigurator()
          .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
      followerMotor
          .getConfigurator()
          .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));
    } else {
      leadMotor
          .getConfigurator()
          .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
      followerMotor
          .getConfigurator()
          .apply(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast));
    }
  }

  @Override
  public void setPID(double p, double i, double d) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kP = p;
    configs.kI = i;
    configs.kD = d;

    leadMotor.getConfigurator().apply(configs);
    followerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    configs.kG = kG;

    leadMotor.getConfigurator().apply(configs);
    followerMotor.getConfigurator().apply(configs);
  }
}
