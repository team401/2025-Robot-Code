package frc.robot.subsystems.climb.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.constants.ClimbConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;

public class LiftingState implements PeriodicStateInterface {

  private ClimbSubsystem climbSubsystem;

  public LiftingState(ClimbSubsystem climbSubsystem) {
    this.climbSubsystem = climbSubsystem;
  }

  @Override
  public void periodic() {
    climbSubsystem.setPID(
        ClimbConstants.synced.getObject().climbkPInc,
        ClimbConstants.synced.getObject().climbkI,
        ClimbConstants.synced.getObject().climbkD);
    climbSubsystem.setGoalAngle(ClimbConstants.synced.getObject().finalHangingAngle);
  }
}
