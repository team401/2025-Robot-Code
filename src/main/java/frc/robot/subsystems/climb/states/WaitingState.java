package frc.robot.subsystems.climb.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbAction;

public class WaitingState implements PeriodicStateInterface {
  private ClimbSubsystem climbSubsystem;

  public WaitingState(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void periodic() {
    climbSubsystem.setFeedforward(0.0);
    climbSubsystem.setGoalAngle(ClimbConstants.synced.getObject().restingAngle);
    if (climbSubsystem.getRampClear()) {
      climbSubsystem.fireTrigger(ClimbAction.CLIMB);
    }
  }
}
