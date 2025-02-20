package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
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
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;

public class WristIOTalonFX implements WristIO {
  TalonFX wristMotor = new TalonFX(JsonConstants.wristConstants.wristMotorId, "canivore");
  CANcoder wristCANcoder = new CANcoder(JsonConstants.wristConstants.wristCANcoderId, "canivore");

  // Keep track of talonFX configs and only update FF/PID when necessary to avoid unnecessary object
  // creation
  private TalonFXConfiguration talonFXConfigs;

  private MutAngle wristGoalPosition = Rotations.mutable(0.2);

  private MotionMagicExpoVoltage request =
      new MotionMagicExpoVoltage(wristGoalPosition).withEnableFOC(true);
  private VoltageOut overrideRequest = new VoltageOut(0.0);

  private boolean motorsDisabled = false;

  private boolean isOverriding = false;
  private MutVoltage overrideVoltage = Volts.mutable(0.0);

  public WristIOTalonFX() {
    talonFXConfigs =
        new TalonFXConfiguration()
            .withFeedback(
                new FeedbackConfigs()
                    .withFeedbackRemoteSensorID(JsonConstants.wristConstants.wristCANcoderId)
                    .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
                    .withSensorToMechanismRatio(
                        JsonConstants.wristConstants.sensorToMechanismRatio))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(JsonConstants.wristConstants.wristNeutralModeValue))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimitEnable(true)
                    .withSupplyCurrentLimit(JsonConstants.wristConstants.wristSupplyCurrentLimit)
                    .withStatorCurrentLimitEnable(true)
                    .withStatorCurrentLimit(JsonConstants.wristConstants.wristStatorCurrentLimit))
            .withTorqueCurrent(
                new TorqueCurrentConfigs()
                    .withPeakForwardTorqueCurrent(JsonConstants.wristConstants.peakFOCCurrent)
                    .withPeakReverseTorqueCurrent(JsonConstants.wristConstants.peakFOCCurrent))
            .withSlot0(
                new Slot0Configs()
                    .withKG(JsonConstants.wristConstants.wristKG)
                    .withKS(JsonConstants.wristConstants.wristKS)
                    .withKV(JsonConstants.wristConstants.wristKV)
                    .withKA(JsonConstants.wristConstants.wristKA)
                    .withKP(JsonConstants.wristConstants.wristKP)
                    .withKI(JsonConstants.wristConstants.wristKI)
                    .withKD(JsonConstants.wristConstants.wristKD)
                    .withGravityType(GravityTypeValue.Arm_Cosine))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(
                        JsonConstants.wristConstants.wristMotionMagicCruiseVelocity)
                    .withMotionMagicExpo_kA(JsonConstants.wristConstants.wristMotionMagicExpo_kA)
                    .withMotionMagicExpo_kV(JsonConstants.wristConstants.wristMotionMagicExpo_kV));

    wristMotor.getConfigurator().apply(talonFXConfigs);

    CANcoderConfiguration ccConfigs =
        new CANcoderConfiguration()
            .withMagnetSensor(
                new MagnetSensorConfigs()
                    .withAbsoluteSensorDiscontinuityPoint(
                        JsonConstants.wristConstants.wristCANcoderAbsoluteSensorDiscontinuityPoint)
                    .withMagnetOffset(JsonConstants.wristConstants.wristCANcoderMagnetOffset)
                    .withSensorDirection(
                        JsonConstants.wristConstants.wristCANcoderSensorDirection));

    wristCANcoder.getConfigurator().apply(ccConfigs);
  }

  public void updateInputs(WristInputs inputs) {
    inputs.wristGoalPosition.mut_replace(wristGoalPosition);
    inputs.wristSetpointPosition.mut_replace(
        Rotations.of(wristMotor.getClosedLoopReference().getValue()));

    StatusSignal<Angle> positionSignal = wristCANcoder.getPosition();
    inputs.isWristEncoderConnected = positionSignal.getStatus().isOK();

    inputs.wristPosition.mut_replace(positionSignal.getValue());
    inputs.wristVelocity.mut_replace(wristCANcoder.getVelocity().getValue());

    inputs.wristTargetVelocity.mut_setMagnitude(
        wristMotor.getClosedLoopReferenceSlope().getValue());

    inputs.wristSupplyCurrent.mut_replace(wristMotor.getSupplyCurrent().getValue());
    inputs.wristStatorCurrent.mut_replace(wristMotor.getStatorCurrent().getValue());
  }

  public void applyOutputs(WristOutputs outputs) {
    if (motorsDisabled) {
      wristMotor.setControl(new VoltageOut(0.0));
      outputs.wristOutput = 0.0;

      return;
    } else if (isOverriding) {
      wristMotor.setControl(overrideRequest.withOutput(overrideVoltage));
      outputs.wristOutput = overrideVoltage.in(Volts);

      return;
    }

    wristMotor.setControl(request.withPosition(wristGoalPosition));
    outputs.wristOutput = wristMotor.getClosedLoopOutput().getValue();
  }

  public void setWristGoalPos(Angle goalPos) {
    wristGoalPosition.mut_replace(goalPos);
  }

  public void setPID(double kP, double kI, double kD) {
    talonFXConfigs.Slot0.kP = kP;
    talonFXConfigs.Slot0.kI = kI;
    talonFXConfigs.Slot0.kD = kD;

    wristMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setMaxProfile(
      AngularVelocity maxVelocity,
      Per<VoltageUnit, AngularAccelerationUnit> expo_kA,
      Per<VoltageUnit, AngularVelocityUnit> expo_kV) {
    talonFXConfigs.withMotionMagic(
        new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(maxVelocity)
            .withMotionMagicExpo_kA(expo_kA)
            .withMotionMagicExpo_kV(expo_kV));

    wristMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setFF(double kS, double kV, double kA, double kG) {
    talonFXConfigs.Slot0.kS = kS;
    talonFXConfigs.Slot0.kV = kV;
    talonFXConfigs.Slot0.kA = kA;
    talonFXConfigs.Slot0.kG = kG;

    wristMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setBrakeMode(boolean brakeMode) {
    if (brakeMode) {
      talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    } else {
      talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    }

    wristMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setCurrentLimits(CurrentLimitsConfigs limits) {
    talonFXConfigs.withCurrentLimits(limits);

    wristMotor.getConfigurator().apply(talonFXConfigs);
  }

  public void setMotorsDisabled(boolean disabled) {
    motorsDisabled = disabled;
  }

  public void setOverrideMode(boolean override) {
    isOverriding = override;
  }

  public void setOverrideVoltage(Voltage volts) {
    overrideVoltage.mut_replace(volts);
  }
}
