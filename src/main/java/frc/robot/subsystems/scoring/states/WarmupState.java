package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class WarmupState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  // Cache measure versions of the Doubles from JSON constants
  private LinearVelocity elevatorStableVelocityMeasure;
  private AngularVelocity wristStableVelocityMeasure;

  Debouncer wristSetpointDebouncer =
      new Debouncer(
          JsonConstants.wristConstants.wristStableDebounceTimeSeconds, DebounceType.kRising);

  public WarmupState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void onEntry(Transition transition) {
    elevatorStableVelocityMeasure =
        MetersPerSecond.of(
            JsonConstants.elevatorConstants.maxElevatorSetpointVelocityMetersPerSecond);
    wristStableVelocityMeasure =
        RotationsPerSecond.of(
            JsonConstants.wristConstants.maxWristSetpointVelocityRotationsPerSecond);
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint;
    if (scoringSubsystem.getGamePiece() == GamePiece.Coral) {
      setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getCoralTarget());

      if (!scoringSubsystem.isCoralDetected()) {
        scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
      }
    } else {
      setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getAlgaeScoreTarget());

      if (!scoringSubsystem.isAlgaeDetected()) {
        scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
      }
    }

    scoringSubsystem.setGoalSetpoint(setpoint);

    if (scoringSubsystem.getGamePiece() == GamePiece.Algae) {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeIdleVoltage);
    } else {
      scoringSubsystem.setClawRollerVoltage(Volts.zero());
    }

    boolean elevatorAtSetpoint =
        scoringSubsystem
            .getElevatorHeight()
            .isNear(
                setpoint.elevatorHeight(), JsonConstants.elevatorConstants.elevatorSetpointEpsilon);

    boolean elevatorStable =
        scoringSubsystem
            .getElevatorVelocity()
            .isNear(MetersPerSecond.zero(), elevatorStableVelocityMeasure);

    boolean wristAtSetpoint =
        scoringSubsystem
            .getWristAngle()
            .isNear(setpoint.wristAngle(), JsonConstants.wristConstants.wristSetpointEpsilon);

    boolean wristAtSetpointDebounced = wristSetpointDebouncer.calculate(wristAtSetpoint);

    boolean wristStable =
        scoringSubsystem
            .getWristVelocity()
            .isNear(RotationsPerSecond.zero(), wristStableVelocityMeasure);

    Logger.recordOutput("scoring/warmup/elevatorAtSetpoint", elevatorAtSetpoint);
    Logger.recordOutput("scoring/warmup/elevatorStable", elevatorStable);
    Logger.recordOutput("scoring/warmup/wristAtSetpoint", wristAtSetpoint);
    Logger.recordOutput("scoring/warmup/wristAtSetpointDebounced", wristAtSetpointDebounced);
    Logger.recordOutput("scoring/warmup/wristStable", wristStable);

    if (elevatorAtSetpoint && elevatorStable && wristAtSetpointDebounced && wristStable) {
      scoringSubsystem.fireTrigger(ScoringTrigger.WarmupReady);
    }
  }
}
