package frc.robot.subsystems.climb.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class IdleState implements PeriodicStateInterface {
  private ClimbSubsystem climbSubsystem;

  public IdleState(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void periodic() {
    climbSubsystem.setFeedforward(0.0);
    climbSubsystem.setGoalAngle(ClimbConstants.synced.getObject().restingAngle);
  }
}
