package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Volts;

import coppercore.parameter_tools.LoggedTunableNumber;
import frc.robot.TestModeManager;
import org.littletonrobotics.junction.Logger;

public class ClawMechanism {
  ClawIO io;
  ClawInputsAutoLogged inputs = new ClawInputsAutoLogged();
  ClawOutputsAutoLogged outputs = new ClawOutputsAutoLogged();

  LoggedTunableNumber manualTuningVolts;

  public ClawMechanism(ClawIO io) {
    manualTuningVolts = new LoggedTunableNumber("ClawTunables/clawManualVolts", 0.0);

    this.io = io;
  }

  /**
   * This method must be called from the subsystem's periodic! Mechanism periodics dont run
   * automatically! *
   */
  public void periodic() {
    io.updateInputs(inputs);
    io.applyOutputs(outputs);

    Logger.processInputs("claw/inputs", inputs);
    Logger.processInputs("claw/outputs", inputs);
  }

  /** This method must be called from the subsystem's test periodic! */
  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case ClawTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (volts) -> {
              io.setVoltage(Volts.of(volts[0]));
            },
            manualTuningVolts);
        break;
      default:
        break;
    }
  }
}
