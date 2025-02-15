package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ElevatorIO.ElevatorOutputMode;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class InitState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  /** Keep track of whether we've moved so we know when we've moved and stopped */
  private boolean hasMovedYet = false;
  private Timer homingTimer = new Timer();

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

    if (scoringSubsystem.getElevatorVelocity().abs(MetersPerSecond) < JsonConstants.elevatorConstants.homingVelocityThreshold.abs(MetersPerSecond)) {
      if (hasMovedYet) {
      // TODO: Seed to zero
      }
    } else {
      hasMovedYet = true;
    };
  }

  @Override 
  public void onExit(Transition transition) {
    scoringSubsystem.setElevatorOverrideMode(ElevatorOutputMode.ClosedLoop);
  }
}
