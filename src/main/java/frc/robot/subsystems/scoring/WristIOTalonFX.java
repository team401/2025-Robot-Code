package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.AngularVelocityUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.WristConstants;
import org.littletonrobotics.junction.Logger;

public class WristIOTalonFX implements WristIO {
  MutAngle wristEncoderGoalAngle = Rotations.mutable(0.0);
  MutAngle wristEncoderSetpointPosition = Rotations.mutable(0.0);

  Current overrideCurrent;
  Voltage overrideVoltage;

  WristOutputMode outputMode = WristOutputMode.ClosedLoop;
  TalonFX wristMotor;

  CANcoder wristEncoder;

  // Reuse the same talonFXConfiguration instead of making a new one each time.
  TalonFXConfiguration talonFXConfigs;

  boolean motorDisabled = false;

  private StatusSignal<Angle> wristEncoderPosition;
  private StatusSignal<AngularVelocity> wristEncoderVelocity;
  private StatusSignal<Current> wristMotorSupplyCurrent;
  private StatusSignal<Current> wristMotorStatorCurrent;

  // Reuse the same motion magic request to avoid garbage collector having to clean them up.
  MotionMagicExpoVoltage motionMagicExpoVoltage = new MotionMagicExpoVoltage(0.0);
  VoltageOut voltageOut = new VoltageOut(0.0);
  TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  public WristIOTalonFX() {
    // Initialize TalonFXs  and CANcoders with their correct IDs
    wristMotor = new TalonFX(WristConstants.synced.getObject().wristMotorId, "canivore");

    wristEncoder = new CANcoder(WristConstants.synced.getObject().wristEncoderId, "canivore");

    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
    cancoderConfiguration.MagnetSensor.withAbsoluteSensorDiscontinuityPoint(
        WristConstants.synced.getObject().wristEncoderAbsoluteSensorDiscontinuityPoint);

    // Update with large CANcoder direction and apply
    cancoderConfiguration.MagnetSensor.SensorDirection =
        WristConstants.synced.getObject().wristEncoderDirection;
    cancoderConfiguration.MagnetSensor.MagnetOffset =
        WristConstants.synced.getObject().wristEncoderMagnetOffset.in(Rotations);
    wristEncoder.getConfigurator().apply(cancoderConfiguration);

    // Cache status signals and refresh them when used
    wristEncoderPosition = wristEncoder.getPosition();
    wristEncoderVelocity = wristEncoder.getVelocity();

    wristMotorSupplyCurrent = wristMotor.getSupplyCurrent();
    wristMotorStatorCurrent = wristMotor.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        wristMotorSupplyCurrent,
        wristMotorStatorCurrent,
        wristEncoderPosition,
        wristEncoderVelocity);

    // Initialize talonFXConfigs to use FusedCANCoder and Motion Magic Expo and have correct PID
    // gains and current limits.
    talonFXConfigs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(wristEncoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(
                        WristConstants.synced.getObject().wristEncoderToMechanismRatio)
                    .withRotorToSensorRatio(
                        WristConstants.synced.getObject().rotorToWristEncoderRatio))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(
                        WristConstants.synced.getObject().wristStatorCurrentLimit))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Arm_Cosine)
                    .withKS(WristConstants.synced.getObject().wristKP)
                    .withKV(WristConstants.synced.getObject().wristKV)
                    .withKA(WristConstants.synced.getObject().wristKA)
                    .withKG(WristConstants.synced.getObject().wristKG)
                    .withKP(WristConstants.synced.getObject().wristKP)
                    .withKI(WristConstants.synced.getObject().wristKI)
                    .withKD(WristConstants.synced.getObject().wristKD)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        WristConstants.synced.getObject()
                            .wristAngularCruiseVelocityRotationsPerSecond)
                    .withMotionMagicExpo_kA(WristConstants.synced.getObject().wristExpo_kA_raw)
                    .withMotionMagicExpo_kV(WristConstants.synced.getObject().wristExpo_kV_raw));

    // Apply talonFX config to motor
    wristMotor.getConfigurator().apply(talonFXConfigs);

    // Make follower motor permanently follow lead motor.
  }

  @Override
  public void updateInputs(WristInputs inputs) {
    StatusCode refreshStatus =
        BaseStatusSignal.refreshAll(wristEncoderPosition, wristEncoderVelocity);

    inputs.wristEncoderPos.mut_replace(wristEncoderPosition.getValue());
    inputs.wristEncoderVel.mut_replace(wristEncoderVelocity.getValue());
    inputs.wristEncoderConnected = refreshStatus.isOK();

    refreshStatus = BaseStatusSignal.refreshAll(wristMotorSupplyCurrent, wristMotorStatorCurrent);

    inputs.wristMotorSupplyCurrent.mut_replace(wristMotorSupplyCurrent.getValue());
    inputs.wristMotorStatorCurrent.mut_replace(wristMotorStatorCurrent.getValue());
    inputs.wristMotorConnected = wristMotor.isConnected();

    inputs.wristEncoderGoalPos.mut_replace(wristEncoderGoalAngle);
    inputs.wristEncoderSetpointPos.mut_replace(wristEncoderSetpointPosition);

    inputs.motionMagicError = wristMotor.getClosedLoopError().getValueAsDouble();

    inputs.wristVelocity.mut_replace(wristEncoder.getVelocity().getValue());
  }

  @Override
  public void applyOutputs(WristOutputs outputs) {
    outputs.motorsDisabled = motorDisabled;
    outputs.outputMode = outputMode;

    motionMagicExpoVoltage.withPosition(wristEncoderGoalAngle);

    if (motorDisabled) {
      wristMotor.setControl(voltageOut.withOutput(0.0));
      outputs.wristAppliedVolts.mut_replace(Volts.of(0.0));
    } else {
      switch (outputMode) {
        case ClosedLoop:
          wristMotor.setControl(motionMagicExpoVoltage);

          wristEncoderSetpointPosition.mut_setMagnitude(
              (wristMotor.getClosedLoopReference().getValue()));

          Logger.recordOutput(
              "wrist/referenceSlope", wristMotor.getClosedLoopReferenceSlope().getValueAsDouble());
          outputs.wristAppliedVolts.mut_replace(
              Volts.of(wristMotor.getClosedLoopOutput().getValueAsDouble()));
          outputs.pContrib.mut_replace(
              Volts.of(wristMotor.getClosedLoopProportionalOutput().getValueAsDouble()));
          outputs.iContrib.mut_replace(
              Volts.of(wristMotor.getClosedLoopIntegratedOutput().getValueAsDouble()));
          outputs.dContrib.mut_replace(
              Volts.of(wristMotor.getClosedLoopDerivativeOutput().getValueAsDouble()));
          break;
        case Voltage:
          wristMotor.setControl(new VoltageOut(overrideVoltage));
          outputs.wristAppliedVolts.mut_replace(overrideVoltage);
          break;
        case Current:
          wristMotor.setControl(currentOut.withOutput(overrideCurrent));
          outputs.wristAppliedVolts.mut_replace(wristMotor.getMotorVoltage().getValue());
          break;
      }
    }
  }

  @Override
  public void setWristEncoderGoalPos(Angle goalPos) {
    wristEncoderGoalAngle.mut_replace(goalPos);
    Logger.recordOutput("wrist/wristEncoderGoalAngle", wristEncoderGoalAngle);
  }

  @Override
  public void setWristEncoderPosition(Angle newAngle) {
    wristEncoder.setPosition(newAngle);
  }

  @Override
  public void setOutputMode(WristOutputMode outputMode) {
    this.outputMode = outputMode;
  }

  @Override
  public void setOverrideVoltage(Voltage volts) {
    overrideVoltage = volts;
  }

  @Override
  public void setOverrideCurrent(Current current) {
    overrideCurrent = current;
  }

  @Override
  public void setPID(double p, double i, double d) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kP = p;
    configs.kI = i;
    configs.kD = d;
    wristMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
    MotionMagicConfigs configs =
        talonFXConfigs
            .MotionMagic
            // .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicExpo_kA(expo_kA)
            .withMotionMagicExpo_kV(expo_kV);
    wristMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setFF(double kS, double kV, double kA, double kG) {
    Slot0Configs configs = talonFXConfigs.Slot0;

    configs.kS = kS;
    configs.kV = kV;
    configs.kA = kA;
    configs.kG = kG;
    wristMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setBrakeMode(boolean brakeMode) {
    wristMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    talonFXConfigs.CurrentLimits.withStatorCurrentLimit(currentLimit);

    // Only apply current limit configs to avoid overwriting PID and FF values from tuning
    wristMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
  }

  @Override
  public void setMotorsDisabled(boolean disabled) {
    motorDisabled = disabled;
  }
}
