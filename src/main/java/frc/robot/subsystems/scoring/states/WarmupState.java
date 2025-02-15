package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotations;
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
    boolean wristAtSetpoint =
        scoringSubsystem.getWristAngle().minus(setpoint.wristAngle()).abs(Rotations)
            <= JsonConstants.wristConstants.wristSetpointEpsilon.in(Rotations);

    Logger.recordOutput("scoring/warmup/elevatorAtSetpoint", elevatorAtSetpoint);
    Logger.recordOutput("scoring/warmup/wristAtSetpoint", wristAtSetpoint);

    if (elevatorAtSetpoint && wristAtSetpoint) {
      scoringSubsystem.fireTrigger(ScoringTrigger.WarmupReady);
    }
  }
}
