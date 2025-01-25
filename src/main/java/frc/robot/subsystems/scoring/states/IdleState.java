package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ElevatorConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class IdleState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public IdleState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    scoringSubsystem.setElevatorGoalHeight(ElevatorConstants.synced.getObject().elevatorIdleHeight);
    scoringSubsystem.setClawRollerVoltage(Volts.zero());
  }
}
