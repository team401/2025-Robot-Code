package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class IntakeState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  // Keep track of these so we can update it in tuning modes
  private static Angle intakeAnglePastCoralRange =
      JsonConstants.clawConstants.intakeAnglePastCoralrange;
  private static Angle intakeAnglePastAlgaeRange =
      JsonConstants.clawConstants.intakeAnglePastAlgaerange;

  /**
   * Keep track of what angle the motor was at when the sensor first detected the game piece.
   *
   * <p>The thinking here is that it will operate like a stopwatch: store the initial angle on
   * detect and then get the current angle to see how much it's rotated since then. r
   */
  private MutAngle detectedAngle = Rotations.mutable(0.0);

  /** Are we currently counting rotations to stop intake */
  private boolean isCounting;

  public IntakeState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  public void onEntry(Transition transition) {
    detectedAngle = Rotations.mutable(0.0);
    isCounting = false;
    Logger.recordOutput("claw/intake/diff", 0.0);
  }

  // @Override
  public void periodic() {
    if (scoringSubsystem.getGamePiece() == GamePiece.Algae) {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.intakeVoltage.times(-1));
    } else {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.intakeVoltage);
    }

    ScoringSetpoint setpoint;
    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        setpoint = JsonConstants.scoringSetpoints.coralStation;
        break;
      case Algae:
        switch (scoringSubsystem.getTarget()) {
          case L2:
            setpoint = JsonConstants.scoringSetpoints.L2algae;
            break;
          case Ground:
            setpoint = JsonConstants.scoringSetpoints.ground;
            break;
          default:
            // TODO: Decide if we want to default to L3 or ground
            System.out.println(
                "ERROR: Scoring commanded to intake algae but FieldTarget was "
                    + scoringSubsystem.getTarget().toString()
                    + " (not L2, L3, or Ground). Defaulted to L3.");
          case L3:
            setpoint = JsonConstants.scoringSetpoints.L3algae;
            break;
        }
        break;
      default:
        System.out.println(
            "ERROR: Unkown GamePiece "
                + scoringSubsystem.getGamePiece()
                + " (defaulted to idle setpoint)");
        setpoint = JsonConstants.scoringSetpoints.idle;
        break;
    }

    scoringSubsystem.setGoalSetpoint(setpoint);

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        if (scoringSubsystem.isCoralDetected()) {
          scoringSubsystem.setClawRollerVoltage(Volts.zero());
          scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);
        }
        break;
      case Algae:
        boolean algaeCurrentDetected = scoringSubsystem.isAlgaeCurrentDetected();

        if (scoringSubsystem.isAlgaeDetected() || algaeCurrentDetected) {
          scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeIdleVoltage);
          scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);

          if (algaeCurrentDetected) {
            scoringSubsystem.setAlgaeCurrentDetected(true);
          }
        }
        break;
    }

    Logger.recordOutput("claw/intake/detectedAngle", detectedAngle);
  }

  public static void setIntakeAnglePastCoralrange(Angle newAngle) {
    intakeAnglePastCoralRange = newAngle;
  }

  public static void setIntakeAnglePastAlgaerange(Angle newAngle) {
    intakeAnglePastAlgaeRange = newAngle;
  }
}
