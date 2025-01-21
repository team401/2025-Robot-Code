package frc.robot.commands.drive;

import coppercore.wpilib_interface.DriveTemplate;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.drive.Drive;

public class DriveWithJoysticksReefscape extends DriveWithJoysticks {
  private CommandJoystick leftJoystick;
  private CommandJoystick rightJoystick;
  private double joystickDeadband;
  private Drive drive;

  public DriveWithJoysticksReefscape(
      Drive drive,
      CommandJoystick leftJoystick,
      CommandJoystick rightJoystick,
      double maxLinearVelocity,
      double maxAngularVelocity,
      double joystickDeadband) {
    super(
        drive,
        leftJoystick,
        rightJoystick,
        maxLinearVelocity,
        maxAngularVelocity,
        joystickDeadband);

    this.leftJoystick = leftJoystick;
    this.rightJoystick = rightJoystick;
    this.joystickDeadband = joystickDeadband;
    this.drive = drive;
  }

  @Override
  public void execute() {
    // NOTE: Joysticks will override OTF here, maybe we want a way to disable that, ie if we hold trigger then continue following
    if (areJoysticksActive()) {
      // set mode to MANUAL and execute joystick command
      drive.setOTF(false);
      super.execute();
    } else {
      // if no OTF, stop drivetrain
      if(!drive.isDriveOTF()) {
        drive.stop();
      }
    }
  }

  /**
   * checks if input is being received from joysticks
   * 
   * @return true if inputs are greater than the deadband on any joystick
   */
  public boolean areJoysticksActive() {
    return Math.abs(leftJoystick.getX()) > joystickDeadband
        || Math.abs(leftJoystick.getY()) > joystickDeadband
        || Math.abs(rightJoystick.getX()) > joystickDeadband;
  }
}
