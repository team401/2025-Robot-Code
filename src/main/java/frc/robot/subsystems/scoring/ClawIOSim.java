package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.JsonConstants;
import org.littletonrobotics.junction.Logger;

public class ClawIOSim implements ClawIO {
  private enum HasState {
    NONE,
    CORAL,
    ALGAE
  }

  boolean hasCoral = false;
  boolean hasAlgae = false;

  boolean coralAvailable = false;
  boolean algaeAvailable = false;

  private MutVoltage outputVoltage = Volts.mutable(0.0);

  // Keep track of what the claw is currently holding
  private HasState has = HasState.NONE;

  // Keep track of how long we've been intaking/outtaking to simulate moving an object in or out
  private Timer movementTimer = new Timer();

  public ClawIOSim() {
    SmartDashboard.putBoolean("clawSim/coralAvailable", false);
    SmartDashboard.putBoolean("clawSim/algaeAvailable", false);

    updateSimState();
  }

  private void updateSimState() {
    if (!DriverStation.isEnabled()) {
      outputVoltage.mut_replace(Volts.zero());
    }

    boolean coralAvailable = SmartDashboard.getBoolean("clawSim/coralAvailable", false);
    boolean algaeAvailable = SmartDashboard.getBoolean("clawSim/algaeAvailable", false);

    if (outputVoltage.in(Volts) == 0.0 || (!coralAvailable && !algaeAvailable)) {
      // Keep movement timer at zero if motors not moving
      // or if no object is available
      movementTimer.restart();
    }

    Logger.recordOutput("clawSim/outputVoltage", outputVoltage.in(Volts));
    Logger.recordOutput("clawSim/movementTimer", movementTimer.get());

    if (movementTimer.get() > JsonConstants.clawConstantsSim.actionTimeSeconds) {
      if (has == HasState.CORAL
          || has == HasState.ALGAE) { // If we had something, we just spit it out
        has = HasState.NONE;
      } else if (has == HasState.NONE
          && coralAvailable) { // If we didn't have anything and we can intake a coral, intake it.
        has = HasState.CORAL;
      } else if (has == HasState.NONE
          && algaeAvailable) { // If we didn't have anything and we can intake an algae, intake it.
        has = HasState.ALGAE;
      }
      movementTimer.restart();
    }

    hasCoral = has == HasState.CORAL;
    hasAlgae = has == HasState.ALGAE;
  }

  private final Current currentPushing = Amps.of(50);

  public void updateInputs(ClawInputs inputs) {
    updateSimState();

    inputs.algaeDetected = hasAlgae;
    inputs.coralDetected = hasCoral;

    Current motorCurrent =
        (hasAlgae || hasCoral || algaeAvailable || coralAvailable)
                && (outputVoltage.in(Volts) != 0.0)
            ? currentPushing
            : Amps.zero();

    inputs.clawStatorCurrent.mut_replace(motorCurrent);
    inputs.clawSupplyCurrent.mut_replace(motorCurrent);
  }

  public void applyOutputs(ClawOutputs outputs) {
    outputs.clawAppliedVolts.mut_replace(outputVoltage);
  }

  public void setVoltage(Voltage volts) {
    outputVoltage.mut_replace(volts);
  }

  public boolean isCoralDetected() {
    return hasCoral;
  }

  public boolean isAlgaeDetected() {
    return hasAlgae;
  }
}
