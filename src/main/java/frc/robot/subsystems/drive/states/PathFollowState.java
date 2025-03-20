package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.subsystems.drive.Drive;

public class PathFollowState implements PeriodicStateInterface {
  private Drive drive;

  public PathFollowState(Drive drive) {
    this.drive = drive;
  }

  public void periodic() {}
}
