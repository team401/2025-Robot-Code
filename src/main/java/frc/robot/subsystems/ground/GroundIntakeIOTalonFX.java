package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.ground.GroundIntakeIO;

public class GroundIntakeIOTalonFX implements GroundIntakeIO {
  TalonFX shoulderMotor = new TalonFX(JsonConstants.groundIntakeConstants.shoulderMotorID);
  TalonFX rollerMotor = new TalonFX(JsonConstants.groundIntakeConstants.rollerMotorID);

  private MutVoltage wristOutputVoltage = Volts.mutable(0.0);
  private MutVoltage rollerOutputVoltage = Volts.mutable(0.0);
  private VoltageOut wristVoltageRequest = new VoltageOut(wristOutputVoltage);
  private VoltageOut rollerVoltageRequest = new VoltageOut(rollerOutputVoltage);

  public GroundIntakeIOTalonFX() {
    TalonFXConfiguration shoulderConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(JsonConstants.groundIntakeConstants.kShoulderMotorInverted))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(JsonConstants.groundIntakeConstants.shoulderMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(JsonConstants.groundIntakeConstants.shoulderMotorStatorCurrentLimit));

    TalonFXConfiguration rollerConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(JsonConstants.groundIntakeConstants.kRollerMotorInverted))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(JsonConstants.groundIntakeConstants.rollerMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(JsonConstants.groundIntakeConstants.rollerMotorStatorCurrentLimit));

    shoulderMotor.getConfigurator().apply(shoulderConfigs);
    rollerMotor.getConfigurator().apply(rollerConfigs);

    shoulderMotor.setNeutralMode(NeutralModeValue.Brake);
    rollerMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  public void updateWristInputs(GroundIntakeInputs inputs) {
    inputs.wristPosition.mut_replace(shoulderMotor.getPosition().getValue());
    inputs.wristVelocity.mut_replace(RotationsPerSecond.of(shoulderMotor.getVelocity().getValueAsDouble()));
    inputs.wristSupplyCurrent.mut_replace(shoulderMotor.getSupplyCurrent() .getValue());
    inputs.wristStatorCurrent.mut_replace(shoulderMotor.getStatorCurrent().getValue());
  }

  public void updateRollerInputs(GroundIntakeInputs inputs) {
    inputs.rollerVelocity.mut_replace(RotationsPerSecond.of(rollerMotor.getVelocity().getValueAsDouble()));
    
    inputs.rollerSupplyCurrent.mut_replace(rollerMotor.getSupplyCurrent().getValue());
    inputs.rollerStatorCurrent.mut_replace(rollerMotor.getStatorCurrent().getValue());
  }

  public void applyWristOutputs(GroundIntakeOutputs outputs) {
    outputs.wristMotorOutput = wristOutputVoltage.in(Volts);
    shoulderMotor.setControl(wristVoltageRequest.withOutput(wristOutputVoltage));
  }

  public void applyRollerOutputs(GroundIntakeOutputs outputs) {
    outputs.rollerMotorOutput = rollerOutputVoltage.in(Volts);
    rollerMotor.setControl(rollerVoltageRequest.withOutput(rollerOutputVoltage));
  }

  public void setWristTargetPosition(Voltage volts) {
    wristOutputVoltage.mut_replace(volts);
  }

  public void setRollerTargetSpeed(AngularVelocity speed) {
    rollerOutputVoltage.mut_replace(Volts.of(speed.in(RotationsPerSecond)));
  }
}
