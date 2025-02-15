package frc.robot;

import static edu.wpi.first.units.Units.*;

import coppercore.vision.VisionLocalizer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.StrategyManager.AutonomyMode;
import frc.robot.commands.drive.AkitDriveCommands;
import frc.robot.constants.AutoStrategy;
import frc.robot.constants.AutoStrategyContainer;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.io.File;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private ScoringSubsystem scoringSubsystem = null;
  private Drive drive = null;
  private ClimbSubsystem climbSubsystem = null;
  private VisionLocalizer vision = null;
  private StrategyManager strategyManager = null;
  private AutoStrategyContainer strategyContainer = null;
  private LED led = null;

  private SendableChooser<AutoStrategy> autoChooser = new SendableChooser<>();

  public static SwerveDriveSimulation driveSim = null;

  // The robot's subsystems and commands are defined here

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    loadConstants();
    configureSubsystems();
    // Configure the trigger bindings
    configureBindings();
    TestModeManager.init();
    configureAutos();
  }

  public void loadConstants() {
    JsonConstants.loadConstants();
    FeatureFlags.synced.loadData();
    OperatorConstants.synced.loadData();
  }

  public void configureAutos() {
    boolean firstDefault = false;
    File autoDirectory =
        new File(Filesystem.getDeployDirectory().toPath().resolve("auto").toString());
    strategyContainer = new AutoStrategyContainer(autoDirectory.listFiles());
    for (AutoStrategy strategy : strategyContainer.getStrategies()) {
      if (!firstDefault) {
        autoChooser.setDefaultOption(strategy.autoStrategyName, strategy);
        firstDefault = true;
      } else {
        autoChooser.addOption(strategy.autoStrategyName, strategy);
      }
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureSubsystems() {
    led = InitSubsystems.initLed();
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
    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem = InitSubsystems.initClimbSubsystem();
    }

    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem = InitSubsystems.initScoringSubsystem();
      if (FeatureFlags.synced.getObject().runDrive) {
        scoringSubsystem.setIsDriveLinedUpSupplier(() -> drive.isDriveAlignmentFinished());
      } else {
        scoringSubsystem.setIsDriveLinedUpSupplier(() -> true);
      }
    }

    strategyManager = new StrategyManager(drive, scoringSubsystem);
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
    if (FeatureFlags.synced.getObject().runClimb) {
      InitBindings.initClimbBindings(climbSubsystem);
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AkitDriveCommands.feedforwardCharacterization(drive);
  }

  public void periodic() {
    strategyManager.periodic();
  }

  public void autonomousInit() {
    strategyManager.setAutonomyMode(AutonomyMode.Full);

    // load chosen strategy
    strategyManager.addActionsFromAutoStrategy(autoChooser.getSelected());
  }

  public void teleopInit() {
    strategyManager.setAutonomyMode(AutonomyMode.Teleop);
    // clear leftover actions from auto
    strategyManager.clearActions();
  }

  /** This method must be called from robot, as it isn't called automatically */
  public void testInit() {
    InitBindings.initTestModeBindings();

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
  }

  public void disabledPeriodic() {
    // Logger.recordOutput("feature_flags/drive", FeatureFlags.synced.getObject().runDrive);
    strategyManager.logActions();
  }

  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();
  }
}
