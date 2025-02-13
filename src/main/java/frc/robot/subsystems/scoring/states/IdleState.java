package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class IdleState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public IdleState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint;

    if (scoringSubsystem.isAlgaeDetected()) {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeIdleVoltage);
      setpoint = JsonConstants.scoringSetpoints.idleWithAlgae;
    } else {
      scoringSubsystem.setClawRollerVoltage(Volts.zero());
      setpoint = JsonConstants.scoringSetpoints.idle;
    }

    scoringSubsystem.setGoalSetpoint(setpoint);
  }
}
