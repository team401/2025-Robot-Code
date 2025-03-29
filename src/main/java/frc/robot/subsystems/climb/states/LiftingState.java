package frc.robot.subsystems.climb.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class LiftingState implements PeriodicStateInterface {

  private ClimbSubsystem climbSubsystem;

  public LiftingState(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void periodic() {
    if (JsonConstants.climbConstants.finalHangingAngle.lt(climbSubsystem.getRotation())) {
      climbSubsystem.setFeedforward(JsonConstants.climbConstants.climbFFVolts);
    } else {
      climbSubsystem.setFeedforward(0.0);
    }

    climbSubsystem.setGoalAngle(JsonConstants.climbConstants.finalHangingAngle);
  }
}
