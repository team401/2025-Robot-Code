package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystemMapleSim;

public class ScoreState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public ScoreState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getTarget());

    scoringSubsystem.setGoalSetpoint(setpoint);

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.coralScoreVoltage);

        if (!scoringSubsystem.isCoralDetected()) {
          ScoringSubsystemMapleSim.shootCoral();
          scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
        }
        break;
      case Algae:
        scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
        if (!scoringSubsystem.isAlgaeDetected()) {
          ScoringSubsystemMapleSim.shootAlgae();
          scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
        }
        break;
    }
  }
}
