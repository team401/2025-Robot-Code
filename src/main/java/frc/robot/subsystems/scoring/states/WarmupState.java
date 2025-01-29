package frc.robot.subsystems.scoring.states;

import static edu.wpi.first.units.Units.Volts;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ScoringSetpoints.ScoringSetpoint;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class WarmupState implements PeriodicStateInterface {
  private ScoringSubsystem scoringSubsystem;

  public WarmupState(ScoringSubsystem scoringSubsystem) {
    this.scoringSubsystem = scoringSubsystem;
  }

  @Override
  public void periodic() {
    ScoringSetpoint setpoint;

    switch (scoringSubsystem.getTarget()) {
      case L1:
        setpoint = JsonConstants.scoringSetpoints.l1;
        break;
      case L2:
        setpoint = JsonConstants.scoringSetpoints.l2;
        break;
      case L3:
        setpoint = JsonConstants.scoringSetpoints.l3;
        break;
      case L4:
        setpoint = JsonConstants.scoringSetpoints.l4;
        break;
      case Net:
        setpoint = JsonConstants.scoringSetpoints.net;
        break;
      case Processor:
        setpoint = JsonConstants.scoringSetpoints.processor;
        break;
      case Ground:
      default:
        System.out.println(
            "ERROR: Can't warmup for FieldTarget "
                + scoringSubsystem.getTarget()
                + ", defaulting to L1");
        setpoint = JsonConstants.scoringSetpoints.l1;
        break;
    }

    scoringSubsystem.setElevatorGoalHeight(setpoint.elevatorHeight());
    scoringSubsystem.setClawRollerVoltage(Volts.zero());
  }
}
