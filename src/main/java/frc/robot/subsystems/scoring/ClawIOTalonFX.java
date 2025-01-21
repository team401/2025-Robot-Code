package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.ClawConstants;

public class ClawIOTalonFX implements ClawIO {
    CANrange coralRange = new CANrange(ClawConstants.synced.getObject().coralCANrangeID);
    CANrange algaeRange = new CANrange(ClawConstants.synced.getObject().algaeCANrangeID);

    TalonFX rollerMotor = new TalonFX(ClawConstants.synced.getObject().clawMotorID);

    private MutVoltage outputVoltage = Volts.mutable(0.0);
    private VoltageOut voltageRequest = new VoltageOut(outputVoltage);

    public ClawIOTalonFX() {
        TalonFXConfiguration talonFXConfigs =
                new TalonFXConfiguration()
                        .withMotorOutput(
                                new MotorOutputConfigs()
                                        .withInverted(
                                                ClawConstants.synced.getObject()
                                                        .kClawMotorInverted))
                        .withCurrentLimits(
                                new CurrentLimitsConfigs()
                                        .withSupplyCurrentLimit(
                                                ClawConstants.synced.getObject()
                                                        .clawSupplyCurrentLimit)
                                        .withStatorCurrentLimit(
                                                ClawConstants.synced.getObject()
                                                        .clawStatorCurrentLimit));

        rollerMotor.getConfigurator().apply(talonFXConfigs);

        CANrangeConfiguration coralRangeConfigs =
                new CANrangeConfiguration()
                        .withProximityParams(
                                new ProximityParamsConfigs()
                                        .withMinSignalStrengthForValidMeasurement(
                                                ClawConstants.synced.getObject()
                                                        .coralMinSignalStrengthForValidMeasurement)
                                        .withProximityThreshold(
                                                ClawConstants.synced.getObject()
                                                        .coralProximityThreshold)
                                        .withProximityHysteresis(
                                                ClawConstants.synced.getObject()
                                                        .coralProximityHysteresis));

        coralRange.getConfigurator().apply(coralRangeConfigs);

        CANrangeConfiguration algaeRangeConfigs =
                new CANrangeConfiguration()
                        .withProximityParams(
                                new ProximityParamsConfigs()
                                        .withMinSignalStrengthForValidMeasurement(
                                                ClawConstants.synced.getObject()
                                                        .algaeMinSignalStrengthForValidMeasurement)
                                        .withProximityThreshold(
                                                ClawConstants.synced.getObject()
                                                        .algaeProximityThreshold)
                                        .withProximityHysteresis(
                                                ClawConstants.synced.getObject()
                                                        .algaeProximityHysteresis));

        algaeRange.getConfigurator().apply(algaeRangeConfigs);
    }

    public void updateInputs(ClawInputs inputs) {}

    public void applyOutputs(ClawOutputs outputs) {
        outputs.clawAppliedVolts.mut_replace(outputVoltage);
        rollerMotor.setControl(voltageRequest.withOutput(outputVoltage));
    }

    public void setVoltage(Voltage volts) {
        outputVoltage.mut_replace(volts);
    }

    public boolean getCoralDetected() {
        return coralRange.getIsDetected().getValue();
    }

    public boolean getAlgaeDetected() {
        return algaeRange.getIsDetected().getValue();
    }
}
