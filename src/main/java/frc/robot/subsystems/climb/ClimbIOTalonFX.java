package frc.robot.subsystems.climb;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.JsonConstants;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.Logger;

public class ClimbIOTalonFX implements ClimbIO {

  TalonFX leadMotor;
  TalonFX followerMotor;
  CANcoder climbAngleCoder;

  TalonFXConfiguration talonFXConfigs;

  // TODO: replace when sensors become available - apparently this is not happening actually but
  // leaving this in case a better solution happens later
  private BooleanSupplier lockedToCage = () -> false;

  private MutAngle goalAngle = Radians.mutable(0);
  private MutVoltage overrideVoltage = Volts.mutable(0.0);

  private boolean override = false;

  private MotionMagicVoltage calculator =
      new MotionMagicVoltage(ClimbConstants.synced.getObject().restingAngle);

  public ClimbIOTalonFX() {
    leadMotor = new TalonFX(16, "canivore");
    followerMotor = new TalonFX(17, "canivore");
    climbAngleCoder = new CANcoder(17, "canivore");

    followerMotor.setControl(
        new Follower(
            leadMotor.getDeviceID(), ClimbConstants.synced.getObject().invertFollowerClimbMotor));

    talonFXConfigs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(climbAngleCoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(JsonConstants.climbConstants.climbInvertValue))
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
    inputs.motorAngle.mut_replace(climbAngleCoder.getAbsolutePosition().getValue());
  }

  @Override
  public void applyOutputs(ClimbOutputs outputs) {

    calculator.withPosition(goalAngle.in(Rotations));

    Logger.recordOutput("climb/calculatorAngle", leadMotor.getPosition().getValueAsDouble());

    if (override) {
      leadMotor.setVoltage(overrideVoltage.in(Volts));
    } else {
      leadMotor.setControl(calculator);
    }
    // tried to replace the above if statement with leadMotor.setControl(new VoltageOut(5));,
    // doesn't work
    // the getMotorVoltage continuously returns 0 despite leadMotor's literal voltage being set to 5
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
  public void setPID(double p, double i, double d) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kP = p;
    configs.kI = i;
    configs.kD = d;

    leadMotor.getConfigurator().apply(configs);
    followerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    leadMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
