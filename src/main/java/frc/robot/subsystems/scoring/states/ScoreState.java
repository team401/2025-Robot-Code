package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class ScoreState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public ScoreState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getTarget());

    scoringSubsystem.setElevatorGoalHeight(setpoint.elevatorHeight());

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.coralScoreVoltage);
        break;
      case Algae:
        scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
        break;
    }
  }
}
