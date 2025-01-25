package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringStateMachineTriggers;

public class IntakeCoralState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public IntakeCoralState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    scoringSubsystem.setElevatorGoalHeight(
        ElevatorConstants.synced.getObject().elevatorIntakeHeight);
    scoringSubsystem.setClawRollerVoltage(ClawConstants.synced.getObject().intakeVoltage);

    if (scoringSubsystem.isCoralDetected()) {
      scoringSubsystem.fireTrigger(ScoringStateMachineTriggers.DoneIntaking);
    }
  }
}
