package frc.robot.subsystems.climb.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbAction;

public class SearchingState implements PeriodicStateInterface {
  private ClimbSubsystem climbSubsystem;

  public SearchingState(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void periodic() {
    climbSubsystem.setGoalAngle(ClimbConstants.synced.getObject().searchingAngle);
    if (climbSubsystem.getLockedToCage()) {
      climbSubsystem.fireTrigger(ClimbAction.CLIMB);
    }
  }
}
