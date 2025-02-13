// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import coppercore.vision.VisionLocalizer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.AkitDriveCommands;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.climb.ClimbSubsystem.ClimbAction;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private ClimbSubsystem climbSubsystem;
  private ScoringSubsystem scoringSubsystem;
  private Drive drive;
  private VisionLocalizer vision;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_buttonMasher = new CommandXboxController(2);

  // The robot's subsystems and commands are defined here

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    loadConstants();
    configureSubsystems();
    // Configure the trigger bindings
    configureBindings();
    TestModeManager.testInit();
  }

  public void loadConstants() {
    JsonConstants.loadConstants();
    FeatureFlags.synced.loadData();
    OperatorConstants.synced.loadData();
  }

  public void configureSubsystems() {
    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem = InitSubsystems.initScoringSubsystem();
    }
    if (FeatureFlags.synced.getObject().runDrive) {
      drive = InitSubsystems.initDriveSubsystem();

      if (FeatureFlags.synced.getObject().runVision) {
        vision = InitSubsystems.initVisionSubsystem(drive);

        drive.setAlignmentSupplier(vision::getDistanceErrorToTag);
      }
    }
    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem = InitSubsystems.initClimbSubsystem();
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
    // initialize helper commands
    if (FeatureFlags.synced.getObject().runDrive) {
      InitBindings.initDriveBindings(drive);
    }
    if (FeatureFlags.synced.getObject().runClimb) {
      InitBindings.initClimbBindings(climbSubsystem);
    }
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
      case DriveFeedForwardCharacterization:
        CommandScheduler.getInstance()
            .schedule(AkitDriveCommands.feedforwardCharacterization(drive));
        break;
      case DriveWheelRadiusCharacterization:
        CommandScheduler.getInstance()
            .schedule(AkitDriveCommands.wheelRadiusCharacterization(drive));
        break;
      case DriveSteerMotorCharacterization:
        CommandScheduler.getInstance()
            .schedule(AkitDriveCommands.steerAngleCharacterization(drive));
        break;

      case DriveSysIdQuasistaticForward:
        CommandScheduler.getInstance()
            .schedule(drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        break;

      case DriveSysIdQuasistaticBackward:
        CommandScheduler.getInstance()
            .schedule(drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
        break;

      case DriveSysIdDynamicForward:
        CommandScheduler.getInstance()
            .schedule(drive.sysIdDynamic(SysIdRoutine.Direction.kForward));
        break;

      case DriveSysIdDynamicBackward:
        CommandScheduler.getInstance()
            .schedule(drive.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        break;
      default:
        break;
    }
  }

  /** This method must be called from the robot, as it isn't called automatically. */
  public void testPeriodic() {
    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem.testPeriodic();
    }
    if (FeatureFlags.synced.getObject().runDrive) {
      drive.testPeriodic();
    }
  }

  public void disabledPeriodic() {
    // Logger.recordOutput("feature_flags/drive", FeatureFlags.synced.getObject().runDrive);
  }

  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
