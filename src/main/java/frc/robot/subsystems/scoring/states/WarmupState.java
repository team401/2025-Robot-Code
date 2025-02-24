package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class WarmupState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public WarmupState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint = ScoringSetpoints.getWarmupSetpoint(scoringSubsystem.getTarget());

    scoringSubsystem.setGoalSetpoint(setpoint);
    scoringSubsystem.setClawRollerVoltage(Volts.zero());

    if (!(scoringSubsystem.isAlgaeDetected() || scoringSubsystem.isCoralDetected())) {
      scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
    }

    boolean elevatorAtSetpoint =
        scoringSubsystem.getElevatorHeight().minus(setpoint.elevatorHeight()).abs(Meters)
            <= JsonConstants.elevatorConstants.elevatorSetpointEpsilon.in(Meters);
    boolean elevatorStable =
        scoringSubsystem.getElevatorVelocity().abs(MetersPerSecond)
            <= JsonConstants.elevatorConstants.maxElevatorSetpointVelocity.in(MetersPerSecond);

    boolean wristAtSetpoint =
        scoringSubsystem.getWristAngle().minus(setpoint.wristAngle()).abs(Rotations)
            <= JsonConstants.wristConstants.wristSetpointEpsilon.in(Rotations);
    boolean wristStable =
        scoringSubsystem.getWristVelocity().abs(RotationsPerSecond)
            <= JsonConstants.wristConstants.maxWristSetpointVelocity.in(RotationsPerSecond);

    Logger.recordOutput("scoring/warmup/elevatorAtSetpoint", elevatorAtSetpoint);
    Logger.recordOutput("scoring/warmup/elevatorStable", elevatorStable);
    Logger.recordOutput("scoring/warmup/wristAtSetpoint", wristAtSetpoint);
    Logger.recordOutput("scoring/warmup/wristStable", wristStable);

    if (elevatorAtSetpoint && elevatorStable && wristAtSetpoint && wristStable) {
      scoringSubsystem.fireTrigger(ScoringTrigger.WarmupReady);
    }
  }
}
