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
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
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
import frc.robot.constants.subsystems.ElevatorConstants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOTalonFX implements ElevatorIO {
  MutAngle largeEncoderGoalAngle = Rotations.mutable(0.0);
  MutAngle largeEncoderSetpointPosition = Rotations.mutable(0.0);
  MutAngle spoolGoalAngle = Rotations.mutable(0.0);

  Current overrideCurrent;
  Voltage overrideVoltage;

  ElevatorOutputMode outputMode = ElevatorOutputMode.ClosedLoop;

  /* TODO: name this left/right or front/back
  I'm unsure of the physical configuration of these motors
  as of yet, so I'm giving them placeholder names
  */
  TalonFX leadMotor;
  TalonFX followerMotor;

  CANcoder largeCANCoder;
  CANcoder smallCANCoder;

  // Reuse the same talonFXConfiguration instead of making a new one each time.
  TalonFXConfiguration talonFXConfigs;

  boolean motorDisabled = false;

  StatusSignal<Angle> largeCANcoderAbsolutePosition;

  // Reuse the same motion magic request to avoid garbage collector having to clean them up.
  MotionMagicExpoTorqueCurrentFOC motionMagicExpoTorqueCurrentFOC =
      new MotionMagicExpoTorqueCurrentFOC(0.0);
  VoltageOut voltageOut = new VoltageOut(0.0);
  TorqueCurrentFOC currentOut = new TorqueCurrentFOC(0.0);

  public ElevatorIOTalonFX() {
    // Initialize TalonFXs  and CANcoders with their correct IDs
    leadMotor = new TalonFX(ElevatorConstants.synced.getObject().leadElevatorMotorId, "canivore");
    followerMotor =
        new TalonFX(ElevatorConstants.synced.getObject().followerElevatorMotorId, "canivore");

    largeCANCoder =
        new CANcoder(ElevatorConstants.synced.getObject().elevatorLargeCANCoderID, "canivore");
    smallCANCoder =
        new CANcoder(ElevatorConstants.synced.getObject().elevatorSmallCANCoderID, "canivore");

    // Create one CANcoder configuration that will be modified slightly and applied to both
    // CANcoders
    CANcoderConfiguration cancoderConfiguration = new CANcoderConfiguration();
    cancoderConfiguration.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
        ElevatorConstants.synced.getObject().elevatorCANCoderDiscontinuityPoint;

    // Update with large CANcoder direction and apply
    cancoderConfiguration.MagnetSensor.SensorDirection =
        ElevatorConstants.synced.getObject().elevatorLargeCANCoderDirection;
    cancoderConfiguration.MagnetSensor.MagnetOffset = -0.026123046875;
    largeCANCoder.getConfigurator().apply(cancoderConfiguration);

    largeCANcoderAbsolutePosition = largeCANCoder.getAbsolutePosition();

    BaseStatusSignal.setUpdateFrequencyForAll(50.0, largeCANcoderAbsolutePosition);

    // Update with small CANcoder direction and apply
    cancoderConfiguration.MagnetSensor.SensorDirection =
        ElevatorConstants.synced.getObject().elevatorSmallCANCoderDirection;
    cancoderConfiguration.MagnetSensor.MagnetOffset = 0.000244;
    smallCANCoder.getConfigurator().apply(cancoderConfiguration);

    // Initialize talonFXConfigs to use FusedCANCoder and Motion Magic Expo and have correct PID
    // gains and current limits.
    talonFXConfigs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(largeCANCoder.getDeviceID())
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(
                        ElevatorConstants.synced.getObject().largeCANCoderToMechanismRatio)
                    .withRotorToSensorRatio(
                        ElevatorConstants.synced.getObject().rotorToLargeCANCoderRatio))
            .withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Coast))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(
                        ElevatorConstants.synced.getObject().elevatorStatorCurrentLimit))
            .withSlot0(
                new Slot0Configs()
                    .withGravityType(GravityTypeValue.Elevator_Static)
                    .withKS(ElevatorConstants.synced.getObject().elevatorkS)
                    .withKV(ElevatorConstants.synced.getObject().elevatorkV)
                    .withKA(ElevatorConstants.synced.getObject().elevatorkA)
                    .withKG(ElevatorConstants.synced.getObject().elevatorkG)
                    .withKP(ElevatorConstants.synced.getObject().elevatorkP)
                    .withKI(ElevatorConstants.synced.getObject().elevatorkI)
                    .withKD(ElevatorConstants.synced.getObject().elevatorkD))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        ElevatorConstants.synced.getObject().elevatorAngularCruiseVelocity)
                    .withMotionMagicExpo_kA(
                        ElevatorConstants.synced.getObject().elevatorExpo_kA_raw)
                    .withMotionMagicExpo_kV(
                        ElevatorConstants.synced.getObject().elevatorExpo_kV_raw));

    // Apply talonFX config to both motors
    leadMotor.getConfigurator().apply(talonFXConfigs);
    followerMotor.getConfigurator().apply(talonFXConfigs);

    // Make follower motor permanently follow lead motor.
    followerMotor.setControl(
        new Follower(
            leadMotor.getDeviceID(),
            ElevatorConstants.synced.getObject().invertFollowerElevatorMotor));
  }

  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.largeEncoderPos.mut_replace(largeCANCoder.getPosition().getValue());
    inputs.smallEncoderPos.mut_replace(smallCANCoder.getPosition().getValue());

    inputs.largeEncoderVel.mut_replace(largeCANCoder.getVelocity().getValue());
    inputs.leadMotorAngle.mut_replace(leadMotor.getPosition().getValue());

    StatusCode refreshStatus = BaseStatusSignal.refreshAll(largeCANcoderAbsolutePosition);

    inputs.largeEncoderConnected = refreshStatus.isOK();
    inputs.largeEncoderAbsolutePos.mut_replace(largeCANcoderAbsolutePosition.getValue());

    inputs.largeAbsPosRot = largeCANCoder.getAbsolutePosition().getValueAsDouble();

    refreshStatus = BaseStatusSignal.refreshAll(smallCANCoder.getAbsolutePosition());

    inputs.smallEncoderConnected = refreshStatus.isOK();
    inputs.smallEncoderAbsolutePos.mut_replace(
        smallCANCoder.getAbsolutePosition().refresh().getValue());

    inputs.largeEncoderGoalPos.mut_replace(largeEncoderGoalAngle);
    inputs.largeEncoderSetpointPos.mut_replace(largeEncoderSetpointPosition);

    inputs.elevatorLeadMotorStatorCurrent.mut_replace(leadMotor.getStatorCurrent().getValue());
    inputs.elevatorLeadMotorSupplyCurrent.mut_replace(leadMotor.getSupplyCurrent().getValue());

    inputs.elevatorFollowerMotorStatorCurrent.mut_replace(
        followerMotor.getStatorCurrent().getValue());
    inputs.elevatorFollowerMotorSupplyCurrent.mut_replace(
        followerMotor.getSupplyCurrent().getValue());

    inputs.motionMagicError = leadMotor.getClosedLoopError().getValueAsDouble();

    inputs.elevatorMechanismVelocity.mut_replace(largeCANCoder.getVelocity().getValue());
  }

  @Override
  public void applyOutputs(ElevatorOutputs outputs) {
    outputs.motorsDisabled = motorDisabled;
    outputs.outputMode = outputMode;

    motionMagicExpoTorqueCurrentFOC.withPosition(largeEncoderGoalAngle);

    if (motorDisabled) {
      leadMotor.setControl(voltageOut.withOutput(0.0));
      outputs.elevatorAppliedVolts.mut_replace(Volts.of(0.0));
    } else {
      switch (outputMode) {
        case ClosedLoop:
          leadMotor.setControl(motionMagicExpoTorqueCurrentFOC);

          largeEncoderSetpointPosition.mut_setMagnitude(
              (leadMotor.getClosedLoopReference().getValue()));

          Logger.recordOutput(
              "elevator/referenceSlope",
              leadMotor.getClosedLoopReferenceSlope().getValueAsDouble());
          outputs.elevatorAppliedVolts.mut_replace(
              Volts.of(leadMotor.getClosedLoopOutput().getValueAsDouble()));
          outputs.pContrib.mut_replace(
              Volts.of(leadMotor.getClosedLoopProportionalOutput().getValueAsDouble()));
          outputs.iContrib.mut_replace(
              Volts.of(leadMotor.getClosedLoopIntegratedOutput().getValueAsDouble()));
          outputs.dContrib.mut_replace(
              Volts.of(leadMotor.getClosedLoopDerivativeOutput().getValueAsDouble()));
          break;
        case Voltage:
          leadMotor.setControl(new VoltageOut(overrideVoltage));
          outputs.elevatorAppliedVolts.mut_replace(overrideVoltage);
          break;
        case Current:
          leadMotor.setControl(currentOut.withOutput(overrideCurrent));
          outputs.elevatorAppliedVolts.mut_replace(leadMotor.getMotorVoltage().getValue());
          break;
      }
    }
  }

  @Override
  public void setLargeCANCoderGoalPos(Angle goalPos) {
    largeEncoderGoalAngle.mut_replace(goalPos);
    spoolGoalAngle.mut_replace(
        goalPos.div(ElevatorConstants.synced.getObject().largeCANCoderToMechanismRatio));
  }

  @Override
  public Angle getLargeCANCoderAbsPos() {
    return largeCANCoder.getAbsolutePosition().getValue();
  }

  @Override
  public Angle getSmallCANCoderAbsPos() {
    return smallCANCoder.getAbsolutePosition().getValue();
  }

  @Override
  public void setLargeCANCoderPosition(Angle newAngle) {
    largeCANCoder.setPosition(newAngle);
  }

  @Override
  public void setOutputMode(ElevatorOutputMode outputMode) {
    this.outputMode = outputMode;
  }

  @Override
  public void setSmallCANCoderPosition(Angle newAngle) {
    smallCANCoder.setPosition(newAngle);
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

    leadMotor.getConfigurator().apply(configs);
    followerMotor.getConfigurator().apply(configs);
  }

  @Override
  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
    // TODO: Figure out how to handle maximum velocity for the elevator

    MotionMagicConfigs configs =
        talonFXConfigs
            .MotionMagic
            // .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicExpo_kA(expo_kA)
            .withMotionMagicExpo_kV(expo_kV);

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

  @Override
  public void setBrakeMode(boolean brakeMode) {
    leadMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerMotor.setNeutralMode(brakeMode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setStatorCurrentLimit(Current currentLimit) {
    talonFXConfigs.CurrentLimits.withStatorCurrentLimit(currentLimit);

    // Only apply current limit configs to avoid overwriting PID and FF values from tuning
    leadMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
    followerMotor.getConfigurator().apply(talonFXConfigs.CurrentLimits);
  }

  @Override
  public void setMotorsDisabled(boolean disabled) {
    motorDisabled = disabled;
  }
}
