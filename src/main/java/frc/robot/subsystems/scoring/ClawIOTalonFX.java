package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Volts;

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
