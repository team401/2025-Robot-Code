package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class InitState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  /** Keep track of whether we've moved so we know when we've moved and stopped */
  private boolean hasMovedYet = false;

  private Timer homingTimer = new Timer();

  private MedianFilter velocityFilter =
      new MedianFilter(JsonConstants.elevatorConstants.homingVelocityFilterWindowSize);

  public InitState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void onEntry(Transition transition) {
    scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.Voltage);
    scoringSubsystem.setElevatorOverrideVoltage(JsonConstants.elevatorConstants.homingVoltage);

    homingTimer.reset();
    homingTimer.start();
  }

  @Override
  public void periodic() {
    if (scoringSubsystem.hasBeenSeeded()) {
      scoringSubsystem.fireTrigger(ScoringTrigger.Seeded);

      return;
    }

    double filteredAbsVelocity =
        velocityFilter.calculate(scoringSubsystem.getElevatorVelocity().abs(MetersPerSecond));

    if (filteredAbsVelocity
        < JsonConstants.elevatorConstants.homingVelocityThresholdMetersPerSecond) {
      // If the elevator is NOT moving:
      if (hasMovedYet) {
        // If it HAS moved yet, that means it's moved and then come to rest, therefore we are at
        // zero
        seedToZero();
      } else if (homingTimer.hasElapsed(
          JsonConstants.elevatorConstants.homingMaxUnmovingTime.in(Seconds))) {
        // If it hasn't moved yet, and it's been longer than our threshold, we're satisfied that it
        // was already at zero
        seedToZero();
      }
    } else {
      // If the elevator IS moving, we keep track of that in hasMovedYet so we'll know when it stops
      hasMovedYet = true;
    }
    ;

    if (homingTimer.hasElapsed(JsonConstants.elevatorConstants.homingMaxTime.in(Seconds))) {
      // If it's been a really long time, give up and say we're at zero
      seedToZero();
    }
  }

  public void seedToZero() {
    scoringSubsystem.seedElevatorToZero();

    scoringSubsystem.fireTrigger(ScoringTrigger.Seeded);
  }

  @Override
  public void onExit(Transition transition) {
    scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.ClosedLoop);
  }
}
