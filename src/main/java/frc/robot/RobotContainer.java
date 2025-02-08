package frc.robot;

import static edu.wpi.first.units.Units.*;

import coppercore.vision.VisionLocalizer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.AkitDriveCommands;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private ScoringSubsystem scoringSubsystem;
  public static  Drive drive;
  private VisionLocalizer vision;

  public static SwerveDriveSimulation driveSim = null;

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
      if (ModeConstants.simMode == frc.robot.constants.ModeConstants.Mode.MAPLESIM) {
        drive.setPose(
            new Pose2d(Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180))));
      }
      if (FeatureFlags.synced.getObject().runVision) {
        vision = InitSubsystems.initVisionSubsystem(drive);

        drive.setAlignmentSupplier(vision::getDistanceErrorToTag);
      }
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    // initialize helper commands
    if (FeatureFlags.synced.getObject().runDrive) {
      InitBindings.initDriveBindings(drive);
    }
  }

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
