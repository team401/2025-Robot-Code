package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.InitBindings;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class ScoreState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public ScoreState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint;

    // Only warmup like normal when we aren't doing algae in the net
    if (scoringSubsystem.getGamePiece() == GamePiece.Algae
        && scoringSubsystem.getAlgaeScoreTarget() == FieldTarget.Net) {
      // Force the net setpoint only in score since the "warmup" for the net stays low to use
      // elevator to shoot algae upward
      setpoint = JsonConstants.scoringSetpoints.net;
    } else if (scoringSubsystem.getGamePiece() == GamePiece.Algae) {
      setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getAlgaeScoreTarget());
    } else {
      setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getCoralTarget());
    }

    scoringSubsystem.setGoalSetpoint(setpoint);

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        if (scoringSubsystem.getCoralTarget() == FieldTarget.L1
            || scoringSubsystem.getCoralTarget() == FieldTarget.L2
            || scoringSubsystem.getCoralTarget() == FieldTarget.L3) {
          scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.coralL23ScoreVoltage);
        } else {
          scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.coralScoreVoltage);
        }

        if (!scoringSubsystem.isCoralDetected()) {
          scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
        }
        break;
      case Algae:
        if (scoringSubsystem.getAlgaeScoreTarget() == FieldTarget.Net) {
          // Only run claw rollers when elevator is at setpoint
          if (scoringSubsystem
              .getWristAngle()
              .isNear(
                  JsonConstants.scoringSetpoints.net.wristAngle(),
                  JsonConstants.wristConstants.netShotRollerWristEpsilon)) {
            scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
          } else {
            scoringSubsystem.setClawRollerVoltage(Volts.zero());
          }
        } else {
          // If scoring algae but not in the net, run the rollers (score processor as normal)
          scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
        }

        if (!scoringSubsystem.isAlgaeDetected()) {
          if (scoringSubsystem.getAlgaeScoreTarget() == FieldTarget.Processor
              && !InitBindings.isWarmupPressed()) {
            scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
          } else if (scoringSubsystem.getAlgaeScoreTarget() != FieldTarget.Processor) {
            scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
          }
        }
        break;
    }
  }
}
