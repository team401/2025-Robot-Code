package frc.robot.commands.drive;

import coppercore.wpilib_interface.DriveTemplate;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DriveWithJoysticksReefscape extends DriveWithJoysticks {
  private CommandJoystick leftJoystick;
  private CommandJoystick rightJoystick;
  private double joystickDeadband;

  public DriveWithJoysticksReefscape(
      DriveTemplate drive,
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
  }

  @Override
  public void execute() {
    if (areJoysticksActive()) {
      // set mode to MANUAL and execute joystick command
      super.execute();
    } else {
      // check for OTF mode, otherwise still run joystick command, or maybe run stop here?
    }
  }

  /**
   * checks if input is being received from joysticks
   * 
   * @return true if inputs are greater than the deadband on any joystick
   */
  public boolean areJoysticksActive() {
    return leftJoystick.getX() > joystickDeadband
        || leftJoystick.getY() > joystickDeadband
        || rightJoystick.getX() > joystickDeadband;
  }
}
