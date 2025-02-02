package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.drive.DesiredLocationSelector;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;

public final class InitBindings {

  @JSONExclude
  public static final JSONSync<InitBindings> synced =
      new JSONSync<InitBindings>(
          new InitBindings(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/InitBingings.json")
              .toString(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  // Controller
  private static final CommandJoystick leftJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kLeftJoystickPort);

  private static final CommandJoystick rightJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kRightJoystickPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.synced.getObject().kDriverControllerPort);

  private Translation2d redReefCenter;
  private Translation2d blueReefCenter;

  public void initBindingValues() {
    redReefCenter = JsonConstants.redFieldLocations.redReefCenterTranslation;
    blueReefCenter = JsonConstants.blueFieldLocations.blueReefCenterTranslation;
  }

  public void initDriveBindings(Drive drive) {
    // Default command, normal field-relative drive
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive, // type: DriveTemplate
            leftJoystick, // type: CommandJoystick
            rightJoystick, // type: CommandJoystick
            JsonConstants.drivetrainConstants.maxLinearSpeed, // type: double (m/s)
            JsonConstants.drivetrainConstants.maxAngularSpeed, // type: double (rad/s)
            JsonConstants.drivetrainConstants.joystickDeadband // type: double
            ));

    // hold right joystick trigger down to have drive go to desired location
    rightJoystick
        .button(1)
        .onTrue(
            new InstantCommand(
                () -> {
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
                  if (DriverStation.getAlliance().isPresent()
                      && DriverStation.getAlliance().get() == Alliance.Red) {
                    drive.alignToFieldElement(redReefCenter);
                  } else {
                    drive.alignToFieldElement(blueReefCenter);
                  }
                },
                drive));
    leftJoystick
        .button(1)
        .onFalse(
            new InstantCommand(
                () -> {
                  drive.disableAlign();
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
