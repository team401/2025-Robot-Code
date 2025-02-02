package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ScoringSetpoints;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class WarmupState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public WarmupState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getTarget());

    scoringSubsystem.setElevatorGoalHeight(setpoint.elevatorHeight());
    scoringSubsystem.setClawRollerVoltage(Volts.zero());

    if (!(scoringSubsystem.isAlgaeDetected() || scoringSubsystem.isCoralDetected())) {
      scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
    }
  }
}
