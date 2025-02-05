package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.wpilib_interface.DriveTemplate;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.subsystems.drive.Drive;

public class JoystickDrive implements PeriodicStateInterface, DriveTemplate {
  private Drive drive;

  public JoystickDrive(Drive drive) {
    this.drive = drive;
  }

  public void periodic() {}

  public void setGoalSpeeds(ChassisSpeeds speeds, boolean fieldCentric) {
    drive.setGoalSpeeds(speeds, true);
  }
}
