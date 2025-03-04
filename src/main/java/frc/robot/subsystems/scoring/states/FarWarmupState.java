package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class FarWarmupState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public FarWarmupState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void onEntry(Transition transition) {}

  @Override
  public void periodic() {
    ScoringSetpoint setpoint = JsonConstants.scoringSetpoints.farWarmup;

    scoringSubsystem.setGoalSetpoint(setpoint);
    scoringSubsystem.setClawRollerVoltage(Volts.zero());

    if (!(scoringSubsystem.isAlgaeDetected() || scoringSubsystem.isCoralDetected())) {
      scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
    }
  }
}
