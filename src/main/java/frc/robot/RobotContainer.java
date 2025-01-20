// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import coppercore.wpilib_interface.DriveWithJoysticks;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.drive.AkitDriveCommands;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DrivetrainConstants;
import frc.robot.subsystems.elevator.ElevatorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private ElevatorSubsystem elevatorSubsystem;
  private Drive drive;

  // Controller
  private final CommandJoystick leftJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kLeftJoystickPort);

  private final CommandJoystick rightJoystick =
      new CommandJoystick(OperatorConstants.synced.getObject().kRightJoystickPort);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.synced.getObject().kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    loadConstants();
    configureSubsystems();
    // Configure the trigger bindings
    configureBindings();
  }

  public void loadConstants() {
    JsonConstants.loadConstants();
    FeatureFlags.synced.loadData();
    OperatorConstants.synced.loadData();
  }

  public void configureSubsystems() {
    if (FeatureFlags.synced.getObject().runElevator) {
      elevatorSubsystem = InitSubsystems.initElevatorSubsystem();
    }
    if (FeatureFlags.synced.getObject().runDrive) {
      drive = InitSubsystems.initDriveSubsystem();
    }
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
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
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AkitDriveCommands.feedforwardCharacterization(drive);
  }

  /** This method must be called from robot, as it isn't called automatically */
  public void testInit() {
    switch (TestModeManager.getTestMode()) {
      case FeedForwardCharacterization:
        CommandScheduler.getInstance()
            .schedule(AkitDriveCommands.feedforwardCharacterization(drive));
        break;
      default:
        break;
    }
  }

  /** This method must be called from the robot, as it isn't called automatically. */
  public void testPeriodic() {
    if (FeatureFlags.synced.getObject().runElevator) {
      elevatorSubsystem.testPeriodic();
    }
  }

  public void disabledPeriodic() {}

  public void disabledInit() {
    TestModeManager.testInit();
  }
}
