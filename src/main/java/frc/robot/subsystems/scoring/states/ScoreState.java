package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class ScoreState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public ScoreState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    scoringSubsystem.setElevatorGoalHeight(JsonConstants.scoringSetpoints.idle.elevatorHeight());
    scoringSubsystem.setClawRollerVoltage(Volts.zero());
  }
}
