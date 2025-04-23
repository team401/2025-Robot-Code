package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class AlgaeScoreState implements PeriodicStateInterface {
  private final ScoringSubsystem scoringSubsystem;

  private boolean hasAlgae;

  public AlgaeScoreState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    hasAlgae = scoringSubsystem.isAlgaeDetected();

    if (!hasAlgae) {
      //leave the state when there is no algae
      scoringSubsystem.fireTrigger(ScoringTrigger.ReturnToIdle);
      return;
    }

    // Set the goal to the Net setpoint
    var setpoint = JsonConstants.scoringSetpoints.net;
    scoringSubsystem.setGoalSetpoint(setpoint);

    // Get current elevator height and wrist angle
    Distance elevatorHeight = scoringSubsystem.getElevatorHeight();
    Angle wristAngle = scoringSubsystem.getWristAngle(); // assuming same units

    // Move wrist when elevator is above a threshold
    if (elevatorHeight.baseUnitMagnitude() > JsonConstants.elevatorConstants.wristFlipHeightThreshold.baseUnitMagnitude()) {
      scoringSubsystem.setWristGoalAngle(setpoint.wristAngle());
    }

    // Spin rollers when wrist is near target
    if (scoringSubsystem
            .getWristAngle()
            .isNear(setpoint.wristAngle(), JsonConstants.wristConstants.netShotRollerWristEpsilon)) {
      scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.algaeScoreVoltage);
    } else {
      scoringSubsystem.setClawRollerVoltage(Volts.zero());
    }
  }
}
