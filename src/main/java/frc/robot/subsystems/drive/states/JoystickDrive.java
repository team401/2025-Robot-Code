package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import frc.robot.subsystems.drive.Drive;

public class JoystickDrive implements PeriodicStateInterface {
  private Drive drive;

  public JoystickDrive(Drive drive) {
    this.drive = drive;
  }
}
