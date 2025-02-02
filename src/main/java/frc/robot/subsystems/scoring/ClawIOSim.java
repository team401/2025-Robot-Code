package frc.robot.subsystems.scoring;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.MutAngle;
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

  boolean coralSensed = false;
  boolean algaeSensed = false;

  boolean coralAvailable = false;
  boolean algaeAvailable = false;

  private MutVoltage outputVoltage = Volts.mutable(0.0);

  /** Keep track of simulated motor position */
  private MutAngle motorPos = Rotations.mutable(0.0);

  /**
   * Keeps track of where a gamepiece is.
   *
   * <ul>
   *   <li>When simulating coral, this value will be initialized to 0.0 and then move to 1.0, where
   *       any value beyond 1.0 is dropping the coral.
   *   <li>When simulating algae, this value will begin at 1.0 and be able to move back down to 0.0.
   *       Moving it out to 1.0 will drop the algae.
   *   <li>When not holding a gamepiece, this value will be -1.0.
   * </ul>
   */
  private double piecePos = -1.0;

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

    // v Volts * k Rotations/Second*Volt * s Seconds =  Rotations
    motorPos.mut_plus(
        RotationsPerSecond.of(
                outputVoltage.in(Volts) * JsonConstants.clawConstantsSim.rotationsPerSecondPerVolt)
            .times(Seconds.of(0.02)));

    if (has == HasState.NONE) {
      if (outputVoltage.in(Volts) > 0.0 && algaeAvailable) {
        has = HasState.ALGAE;
        piecePos = 1.0;
      }
      if (outputVoltage.in(Volts) > 0.0 && coralAvailable) {
        has = HasState.CORAL;
        piecePos = 0.0;
      }
    } else {
      piecePos +=
          RotationsPerSecond.of(
                      outputVoltage.in(Volts)
                          * JsonConstants.clawConstantsSim.rotationsPerSecondPerVolt)
                  .times(Seconds.of(0.02))
                  .in(Rotations)
              * 0.05
              * (has == HasState.ALGAE ? -1.0 : 1.0);

      if (piecePos > 1.0) {
        has = HasState.NONE;
        piecePos = -1.0;
      }

      if (piecePos < 0.0) {
        if (has == HasState.ALGAE) {
          piecePos = 0.0;
        } else if (has == HasState.CORAL) {
          has = HasState.NONE;
          piecePos = -1.0;
        }
      }
    }

    Logger.recordOutput("clawSim/outputVoltage", outputVoltage.in(Volts));

    hasCoral = has == HasState.CORAL;
    hasAlgae = has == HasState.ALGAE;

    switch (has) {
      case NONE:
        break;
      case CORAL:
        coralSensed = piecePos > JsonConstants.clawConstantsSim.coralDetectionPoint;
        algaeSensed = false;
        break;
      case ALGAE:
        algaeSensed = piecePos < JsonConstants.clawConstantsSim.algaeDetectionPoint;
        coralSensed = false;
        break;
    }

    Logger.recordOutput("clawSim/hasCoral", hasCoral);
    Logger.recordOutput("clawSim/hasAlgae", hasAlgae);
    Logger.recordOutput("clawSim/piecePos", piecePos);
  }

  private final Current currentPushing = Amps.of(50);

  public void updateInputs(ClawInputs inputs) {
    updateSimState();

    inputs.algaeDetected = algaeSensed;
    inputs.coralDetected = coralSensed;

    Current motorCurrent =
        (hasAlgae || hasCoral || algaeAvailable || coralAvailable)
                && (outputVoltage.in(Volts) != 0.0)
            ? currentPushing
            : Amps.zero();

    inputs.clawMotorPos.mut_replace(motorPos);
    inputs.clawStatorCurrent.mut_replace(motorCurrent);
    inputs.clawSupplyCurrent.mut_replace(motorCurrent);
  }

  public void applyOutputs(ClawOutputs outputs) {
    outputs.clawAppliedVolts.mut_replace(outputVoltage);
  }

  public void setVoltage(Voltage volts) {
    outputVoltage.mut_replace(volts);
  }

  public Angle getClawMotorPos() {
    return motorPos;
  }

  public boolean isCoralDetected() {
    return coralSensed;
  }

  public boolean isAlgaeDetected() {
    return algaeSensed;
  }
}
