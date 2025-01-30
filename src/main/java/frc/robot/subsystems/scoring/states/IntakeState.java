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
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class IntakeState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  /**
   * Keep track of what angle the motor was at when the sensor first detected the game piece.
   *
   * <p>The thinking here is that it will operate like a stopwatch: store the initial angle on
   * detect and then get the current angle to see how much it's rotated since then. r
   */
  private MutAngle detectedAngle;

  /** Are we currently counting rotations to stop intake */
  private boolean isCounting;

  public IntakeState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  public void onEntry(Transition transition) {
    detectedAngle = Rotations.mutable(0.0);
    isCounting = false;
  }

  // @Override
  public void periodic() {
    scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.intakeVoltage);

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
      default:
        System.out.println(
            "ERROR: Unkown GamePiece "
                + scoringSubsystem.getGamePiece()
                + " (defaulted to idle setpoint)");
        setpoint = JsonConstants.scoringSetpoints.idle;
        break;
    }

    scoringSubsystem.setElevatorGoalHeight(setpoint.elevatorHeight());

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        if (scoringSubsystem.isCoralDetected()) {
          Angle currentPos = scoringSubsystem.getClawRollerPosition();
          if (isCounting) {
            // If we're already counting, check if we've rotated far enough
            Angle diff = currentPos.minus(detectedAngle);
            if (diff.gt(JsonConstants.clawConstants.intakeAnglePastCoralrange)) {
              scoringSubsystem.setClawRollerVoltage(Volts.zero());
              scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);
            }
          } else {
            // If we're not already counting, start counting and record where we started
            detectedAngle.mut_replace(currentPos);
            isCounting = true;
          }
        } else {
          // If gamepiece not detected, we're no longer counting
          isCounting = false;
        }
        break;
      case Algae:
        if (scoringSubsystem.isAlgaeDetected()) {
          scoringSubsystem.setClawRollerVoltage(Volts.zero());
          Angle currentPos = scoringSubsystem.getClawRollerPosition();
          if (isCounting) {
            Angle diff = currentPos.minus(detectedAngle);
            if (diff.gt(JsonConstants.clawConstants.intakeAnglePastAlgaerange)) {
              scoringSubsystem.setClawRollerVoltage(Volts.zero());
              scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);
            }
          } else {
            detectedAngle.mut_replace(currentPos);
            isCounting = true;
          }
        } else {
          // If gamepiece not detected, we're no longer counting
          isCounting = false;
        }
        break;
    }
  }
}
