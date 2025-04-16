package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;

public class ClawIOTalonFX implements ClawIO {
  CANrange coralRange = new CANrange(JsonConstants.clawConstants.coralCANrangeID, "canivore");
  CANrange algaeRange = new CANrange(JsonConstants.clawConstants.algaeCANrangeID, "canivore");

  TalonFX rollerMotor = new TalonFX(JsonConstants.clawConstants.clawMotorID, "canivore");

  private MutVoltage outputVoltage = Volts.mutable(0.0);
  private VoltageOut voltageRequest = new VoltageOut(outputVoltage);

  public ClawIOTalonFX() {
    TalonFXConfiguration talonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(JsonConstants.clawConstants.kClawMotorInverted)
                    .withNeutralMode(NeutralModeValue.Brake))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(JsonConstants.clawConstants.clawSupplyCurrentLimit)
                    .withStatorCurrentLimit(JsonConstants.clawConstants.clawStatorCurrentLimit));

    rollerMotor.getConfigurator().apply(talonFXConfigs);

    rollerMotor.setNeutralMode(NeutralModeValue.Brake);

    CANrangeConfiguration coralRangeConfigs =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withMinSignalStrengthForValidMeasurement(
                        JsonConstants.clawConstants.coralMinSignalStrengthForValidMeasurement)
                    .withProximityThreshold(JsonConstants.clawConstants.coralProximityThreshold)
                    .withProximityHysteresis(JsonConstants.clawConstants.coralProximityHysteresis));

    coralRange.getConfigurator().apply(coralRangeConfigs);

    CANrangeConfiguration algaeRangeConfigs =
        new CANrangeConfiguration()
            .withProximityParams(
                new ProximityParamsConfigs()
                    .withMinSignalStrengthForValidMeasurement(
                        JsonConstants.clawConstants.algaeMinSignalStrengthForValidMeasurement)
                    .withProximityThreshold(JsonConstants.clawConstants.algaeProximityThreshold)
                    .withProximityHysteresis(JsonConstants.clawConstants.algaeProximityHysteresis));

    algaeRange.getConfigurator().apply(algaeRangeConfigs);
  }

  public void updateInputs(ClawInputs inputs) {
    inputs.algaeDetected = isAlgaeDetected();
    inputs.coralDetected = isCoralDetected();

    inputs.algaeSignalStrength = algaeRange.getSignalStrength().getValueAsDouble();
    inputs.algaeDistance.mut_replace(algaeRange.getDistance().getValue());

    inputs.algaeRangeConnected =
        algaeRange.isConnected()
            && StatusSignal.isAllGood(
                algaeRange.getIsDetected(),
                algaeRange.getSignalStrength(),
                algaeRange.getDistance());

    inputs.coralSignalStrength = coralRange.getSignalStrength().getValueAsDouble();
    inputs.coralDistance.mut_replace(coralRange.getDistance().getValue());

    inputs.coralRangeConnected =
        coralRange.isConnected()
            && StatusSignal.isAllGood(
                coralRange.getIsDetected(),
                coralRange.getSignalStrength(),
                coralRange.getDistance());

    inputs.clawMotorPos.mut_replace(rollerMotor.getPosition().getValue());

    inputs.clawStatorCurrent.mut_replace(rollerMotor.getStatorCurrent().getValue());
    inputs.clawSupplyCurrent.mut_replace(rollerMotor.getSupplyCurrent().getValue());
  }

  public void applyOutputs(ClawOutputs outputs) {
    outputs.clawAppliedVolts.mut_replace(outputVoltage);
    rollerMotor.setControl(voltageRequest.withOutput(outputVoltage));
  }

  public void setVoltage(Voltage volts) {
    outputVoltage.mut_replace(volts);
  }

  public Angle getClawMotorPos() {
    return rollerMotor.getPosition().getValue();
  }

  public boolean isCoralDetected() {
    return coralRange.getIsDetected().getValue();
  }

  public boolean isAlgaeDetected() {
    return algaeRange.getIsDetected().getValue();
  }
}
