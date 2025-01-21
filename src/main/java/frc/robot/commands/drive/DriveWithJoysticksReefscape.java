package frc.robot.commands.drive;

import coppercore.wpilib_interface.DriveTemplate;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;

public class DriveWithJoysticksReefscape extends DriveWithJoysticks {
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
  }
}
