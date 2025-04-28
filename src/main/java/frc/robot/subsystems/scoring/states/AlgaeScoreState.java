package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.InitBindings;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class AlgaeScoreState implements PeriodicStateInterface {
  private final ScoringSubsystem scoringSubsystem;
  private boolean flickLaunched = false;

  public AlgaeScoreState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    if (!scoringSubsystem.isAlgaeDetected()) {
      scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
      return;
    }

    FieldTarget target = scoringSubsystem.getAlgaeScoreTarget();

    if (target == FieldTarget.Net) {
  
      scoringSubsystem.setElevatorGoalHeight(JsonConstants.scoringSetpoints.net.elevatorHeight());

      if (!flickLaunched
          && scoringSubsystem
              .getElevatorHeight()
              .isNear(
                  JsonConstants.scoringSetpoints.net.elevatorHeight(),
                  JsonConstants.elevatorConstants.netScoreSetpointEpsilon)) {
        if (scoringSubsystem
            .getWristAngle()
            .isNear(
                JsonConstants.scoringSetpoints.netWarmup.wristAngle(),
                JsonConstants.wristConstants.netShotRollerWristEpsilon)) {
          scoringSubsystem.setWristGoalAngle(JsonConstants.scoringSetpoints.net.wristAngle());
          scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
          flickLaunched = true;
        } else {
          scoringSubsystem.setClawRollerVoltage(Volts.zero());
        }
      }
    } else {
      
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
    }

  
    if (!scoringSubsystem.isAlgaeDetected()) {
      if (target == FieldTarget.Processor && !InitBindings.isWarmupPressed()) {
        scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
      } else if (target != FieldTarget.Processor) {
        scoringSubsystem.fireTrigger(ScoringTrigger.ScoredPiece);
      }
    }
  }
}
