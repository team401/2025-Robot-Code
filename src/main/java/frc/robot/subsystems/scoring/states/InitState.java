package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

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
    System.out.println("The voltage:" + JsonConstants.elevatorConstants.homingVoltage);
    scoringSubsystem.setElevatorOverrideVoltage(JsonConstants.elevatorConstants.homingVoltage);
    scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.Voltage);

    homingTimer.reset();
    homingTimer.start();
  }

  @Override
  public void periodic() {
    scoringSubsystem.setWristGoalAngle(JsonConstants.scoringSetpoints.idle.wristAngle());

    if (!DriverStation.isEnabled()) {
      homingTimer.restart();
      return;
    }

    scoringSubsystem.setElevatorOverrideVoltage(JsonConstants.elevatorConstants.homingVoltage);
    scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.Voltage);

    if (scoringSubsystem.hasBeenSeeded()) {
      scoringSubsystem.fireTrigger(ScoringTrigger.Seeded);

      return;
    }

    double filteredAbsVelocity =
        velocityFilter.calculate(scoringSubsystem.getElevatorVelocity().abs(MetersPerSecond));

    Logger.recordOutput("scoring/init/filteredAbsVelocity", filteredAbsVelocity);
    Logger.recordOutput("scoring/init/hasMovedYet", hasMovedYet);
    Logger.recordOutput("scoring/init/homingTimer", homingTimer.get());

    if (filteredAbsVelocity
        < JsonConstants.elevatorConstants.homingVelocityThresholdMetersPerSecond) {
      // If the elevator is NOT moving:
      if (hasMovedYet) {
        // If it HAS moved yet, that means it's moved and then come to rest, therefore we are at
        // zero
        System.out.println("Homed by moving then stopping");
        seedToZero();
      } else if (homingTimer.hasElapsed(
          JsonConstants.elevatorConstants.homingMaxUnmovingTime.in(Seconds))) {
        // If it hasn't moved yet, and it's been longer than our threshold, we're satisfied that it
        // was already at zero
        System.out.println("Homed by never moving");
        seedToZero();
      }
    } else {
      // If the elevator IS moving, we keep track of that in hasMovedYet so we'll know when it stops
      hasMovedYet = true;
    }

    if (homingTimer.hasElapsed(JsonConstants.elevatorConstants.homingMaxTime.in(Seconds))) {
      // If it's been a really long time, give up and say we're at zero
      System.out.println("Homed by giving up");
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
