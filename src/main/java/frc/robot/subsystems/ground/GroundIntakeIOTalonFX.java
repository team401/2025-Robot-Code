package frc.robot.subsystems.ground;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.units.measure.MutCurrent;
import edu.wpi.first.units.measure.MutVoltage; 
import edu.wpi.first.units.measure.Voltage;
import frc.robot.constants.JsonConstants;

public class GroundIntakeIOTalonFX implements GroundIntakeIO{
    TalonFX rollerMotor = new TalonFX(JsonConstants.groundIntakeConstants.rollerMotorID);
    TalonFX shoulderMotor = new TalonFX(JsonConstants.groundIntakeConstants.shoulderMotorID);

    private MutVoltage outputVoltage = Volts.mutable(0.0);
    private MutCurrent outputCurrent = Amps.mutable(0.0);
    private VoltageOut voltageRequest = new VoltageOut(outputVoltage);
    
    private MutAngle goalAngle = Radians.mutable(0.0);

    public GroundIntakeIOTalonFX() {
        TalonFXConfiguration shoulderTalonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(JsonConstants.groundIntakeConstants.kShoulderMotorInverted))
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(JsonConstants.groundIntakeConstants.shoulderMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(JsonConstants.groundIntakeConstants.shoulderMotorStatorCurrentLimit));

      shoulderMotor.getConfigurator().apply(shoulderTalonFXConfigs);

      shoulderMotor.setNeutralMode(NeutralModeValue.Brake);

        TalonFXConfiguration rollerTalonFXConfigs =
        new TalonFXConfiguration()
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withInverted(JsonConstants.groundIntakeConstants.kShoulderMotorInverted))
            
            .withCurrentLimits(
                new CurrentLimitsConfigs()
                    .withSupplyCurrentLimit(JsonConstants.groundIntakeConstants.rollerMotorSupplyCurrentLimit)
                    .withStatorCurrentLimit(JsonConstants.groundIntakeConstants.rollerMotorStatorCurrentLimit));
      rollerMotor.getConfigurator().apply(rollerTalonFXConfigs);

      rollerMotor.setNeutralMode(NeutralModeValue.Brake);
    }

    public void updateInputs(GroundIntakeInputs inputs) {

    inputs.shoulderMotorPos.mut_replace(shoulderMotor.getPosition().getValue());

    inputs.shoulderMotorStatorCurrent.mut_replace(shoulderMotor.getStatorCurrent().getValue());
    inputs.shoulderMotorSupplyCurrent.mut_replace(shoulderMotor.getSupplyCurrent().getValue());

  }

  public void applyOutputs(GroundIntakeOutputs outputs) {
    outputs.shoulderAppliedCurrent.mut_replace(outputCurrent);
    shoulderMotor.setControl(voltageRequest.withOutput(outputVoltage));
  }

  public void setVoltage(Voltage volts) {
    outputVoltage.mut_replace(volts);
  }

  @Override
  public void setShoulderGoalPosition(Angle goalAngle){
    this.goalAngle.mut_replace(goalAngle);
  }

}
