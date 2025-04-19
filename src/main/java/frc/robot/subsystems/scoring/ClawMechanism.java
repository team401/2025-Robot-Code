package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import coppercore.parameter_tools.LoggedTunableNumber;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.states.IntakeState;
import org.littletonrobotics.junction.Logger;

public class ClawMechanism {
  private ClawIO io;
  private ClawInputsAutoLogged inputs = new ClawInputsAutoLogged();
  private ClawOutputsAutoLogged outputs = new ClawOutputsAutoLogged();

  private LoggedTunableNumber manualTuningVolts;
  private LoggedTunableNumber coralOvershootRotations;
  private LoggedTunableNumber algaeOvershootRotations;

  private Debouncer algaeRiseDebouncer =
      new Debouncer(
          JsonConstants.clawConstants.algaeDetectionTimeRising.in(Seconds), DebounceType.kRising);

  public ClawMechanism(ClawIO io) {
    manualTuningVolts = new LoggedTunableNumber("ClawTunables/clawManualVolts", 0.0);
    coralOvershootRotations =
        new LoggedTunableNumber(
            "ClawTunables/coralOvershootRotations",
            JsonConstants.clawConstants.intakeAnglePastCoralrange.in(Rotations));
    algaeOvershootRotations =
        new LoggedTunableNumber(
            "ClawTunables/algaeOvershootRotations",
            JsonConstants.clawConstants.intakeAnglePastAlgaerange.in(Rotations));

    this.io = io;
  }

  /**
   * Return a the ClawMechanism instance's ClawIO instance
   *
   * <p>NOTE: Be careful to only call methods from where they're allowed to be called!
   *
   * @return a ClawIO
   */
  public ClawIO getIO() {
    return io;
  }

  /**
   * This method must be called from the subsystem's periodic! Mechanism periodics dont run
   * automatically! *
   */
  public void periodic() {
    io.updateInputs(inputs);
    io.applyOutputs(outputs);

    Logger.processInputs("claw/inputs", inputs);
    Logger.processInputs("claw/outputs", outputs);
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
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (rotations) -> {
              IntakeState.setIntakeAnglePastCoralrange(Rotations.of(rotations[0]));
            },
            coralOvershootRotations);

        LoggedTunableNumber.ifChanged(
            hashCode(),
            (rotations) -> {
              IntakeState.setIntakeAnglePastAlgaerange(Rotations.of(rotations[0]));
            },
            algaeOvershootRotations);

        break;
      default:
        break;
    }
  }

  /**
   * Check whether the claw currently detects a coral
   *
   * @return True if detected, false if not
   */
  public boolean isCoralDetected() {
    return inputs.coralDetected;
  }

  /**
   * Check whether the claw currently detects an algae
   *
   * @return True if detected, false if not
   */
  public boolean isAlgaeDetected() {
    return algaeRiseDebouncer.calculate(inputs.algaeDetected);
  }
}
