package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DesiredLocationSelector;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.DrivetrainConstants;

public final class InitBindings {
  // Controller
  private static final CommandJoystick leftJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kLeftJoystickPort);

  private static final CommandJoystick rightJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kRightJoystickPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.synced.getObject().kDriverControllerPort);

  public static void initDriveBindings(Drive drive) {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, // type: DriveTemplate
            leftJoystick, // type: CommandJoystick
            rightJoystick, // type: CommandJoystick
            DrivetrainConstants.maxLinearSpeed, // type: double (m/s)
            DrivetrainConstants.maxAngularSpeed, // type: double (rad/s)
            DrivetrainConstants.joystickDeadband // type: double
            ));

    // hold right joystick trigger down to have drive go to desired location
    rightJoystick
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setDesiredLocation(DesiredLocation.CoralStationRight);
                  drive.setOTF(true);
                },
                drive));
    rightJoystick
        .button(1)
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.setOTF(false);
                },
                drive));

    leftJoystick
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.lockRotationToPosition(new Translation2d(3.5, 3));
                },
                drive));
    leftJoystick
        .button(1)
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.disableRotationLock();
                },
                drive)); // pov right (reef 0-11 -> processor left -> processor right )
    // pov left (goes backwards of right)
    driverController
        .povRight()
        .onTrue(
            new InstantCommand(
                () -> {
                  DesiredLocationSelector.incrementIndex();
                  DesiredLocationSelector.setLocationFromIndex(drive);
                }));
    driverController
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  DesiredLocationSelector.decrementIndex();
                  DesiredLocationSelector.setLocationFromIndex(drive);
                },
                drive));

    leftJoystick
        .top()
        .onTrue(
            new InstantCommand(
                () -> {
                  drive.setPose(
                      new Pose2d(
                          Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180))));
                }));
  }
}
