package frc.robot.subsystems.climb.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbAction;

public class SearchingState implements PeriodicStateInterface {
  private ClimbSubsystem climbSubsystem;

  public SearchingState(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void periodic() {
    if (climbSubsystem
        .getRotation()
        .plus(JsonConstants.climbConstants.searchingFFMargin)
        .lt(JsonConstants.climbConstants.searchingAngle)) {
      climbSubsystem.setFeedforward(JsonConstants.climbConstants.searchingFFVolts);
    } else {
      climbSubsystem.setFeedforward(0.0);
    }

    climbSubsystem.setGoalAngle(ClimbConstants.synced.getObject().searchingAngle);
    if (climbSubsystem.getLockedToCage()) {
      climbSubsystem.fireTrigger(ClimbAction.CLIMB);
    }
  }
}
