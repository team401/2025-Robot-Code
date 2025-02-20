package frc.robot.subsystems.scoring.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class TuningState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public TuningState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    if (!DriverStation.isTest()) {}
  }
}
