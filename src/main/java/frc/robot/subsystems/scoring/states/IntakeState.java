package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class IntakeState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public IntakeState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    scoringSubsystem.setClawRollerVoltage(JsonConstants.clawConstants.intakeVoltage);

    ScoringSetpoint setpoint;
    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        setpoint = JsonConstants.scoringSetpoints.coralStation;
        break;
      case Algae:
        switch (scoringSubsystem.getTarget()) {
          case L2:
            setpoint = JsonConstants.scoringSetpoints.L2algae;
            break;
          case Ground:
            setpoint = JsonConstants.scoringSetpoints.ground;
            break;
          default:
            // TODO: Decide if we want to default to L3 or ground
            System.out.println(
                "ERROR: Scoring commanded to intake algae but FieldTarget was "
                    + scoringSubsystem.getTarget().toString()
                    + " (not L2, L3, or Ground). Defaulted to L3.");
          case L3:
            setpoint = JsonConstants.scoringSetpoints.L3algae;
            break;
        }
      default:
        System.out.println(
            "ERROR: Unkown GamePiece "
                + scoringSubsystem.getGamePiece()
                + " (defaulted to idle target)");
        setpoint = JsonConstants.scoringSetpoints.idle;
        break;
    }

    scoringSubsystem.setElevatorGoalHeight(setpoint.elevatorHeight());

    switch (scoringSubsystem.getGamePiece()) {
      case Coral:
        if (scoringSubsystem.isCoralDetected()) {
          scoringSubsystem.setClawRollerVoltage(Volts.zero());
          scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);
        }
        break;
      case Algae:
        if (scoringSubsystem.isAlgaeDetected()) {
          scoringSubsystem.setClawRollerVoltage(Volts.zero());
          scoringSubsystem.fireTrigger(ScoringTrigger.DoneIntaking);
        }
        break;
    }
  }
}
