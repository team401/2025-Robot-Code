package frc.robot;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import coppercore.vision.VisionLocalizer;
import coppercore.wpilib_interface.tuning.TuneS;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.drive.AkitDriveCommands;
import frc.robot.constants.AutoStrategy;
import frc.robot.constants.AutoStrategyContainer;
import frc.robot.constants.ClimbConstants;
import frc.robot.constants.FeatureFlags;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.OperatorConstants;
import frc.robot.subsystems.climb.ClimbSubsystem;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.ramp.RampSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.io.File;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.littletonrobotics.junction.Logger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here
  private RampSubsystem rampSubsystem = null;
  private ScoringSubsystem scoringSubsystem = null;
  private Drive drive = null;
  private ClimbSubsystem climbSubsystem = null;
  private VisionLocalizer vision = null;
  private LED led = null;
  private StrategyManager strategyManager = null;
  private AutoStrategyContainer strategyContainer = null;
  private DigitalInput ledSwitch = new DigitalInput(8);
  private DigitalInput brakeSwitch = new DigitalInput(9);

  private SendableChooser<AutoStrategy> autoChooser = new SendableChooser<>();

  private boolean lastBrakeSwitchValue = brakeSwitch.get();
  private boolean lastLedSwitchValue = ledSwitch.get();

  public static SwerveDriveSimulation driveSim = null;

  public void checkSwitchForDisabled() {
    boolean brake = brakeSwitch.get();

    if (brake == lastBrakeSwitchValue) {
      return;
    }

    lastBrakeSwitchValue = brake;
    if (FeatureFlags.synced.getObject().runDrive) {
      drive.setBrakeMode(brake);
    }
    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem.setBrakeMode(brake);
    }
    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem.setBrakeMode(brake);
    }
    if (FeatureFlags.synced.getObject().runRamp) {
      rampSubsystem.setBrakeMode(brake);
    }
  }

  public void checkLedSwitch() {
    boolean ledSwitchValue = ledSwitch.get();

    if (ledSwitchValue != lastLedSwitchValue) {
      lastLedSwitchValue = ledSwitchValue;
      if (FeatureFlags.synced.getObject().runLEDs) {
        led.setLedOn(ledSwitchValue);
      }
    }
  }

  public void setSubsystemsToBrake() {
    if (FeatureFlags.synced.getObject().runDrive) {
      drive.setBrakeMode(true);
    }
    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem.setBrakeMode(true);
    }
    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem.setBrakeMode(true);
    }
    if (FeatureFlags.synced.getObject().runRamp) {
      rampSubsystem.setBrakeMode(true);
    }
  }

  public void updateRobotModel() {
    double height = 0.0;
    double claw_rotation = 0.0;
    double ramp_rotation = 0.0;
    double climb_rotation = 0.0;
    if (scoringSubsystem != null) {
      height = scoringSubsystem.getElevatorHeight().magnitude();
      claw_rotation = scoringSubsystem.getWristAngle().in(Radians);
    }
    if (climbSubsystem != null) {
      climb_rotation = climbSubsystem.getRotation().magnitude();
    }
    if (rampSubsystem != null) {
      ramp_rotation = rampSubsystem.getPosition();
    }
    height = Math.min(height, 1.87);
    double stage_one_height = Math.max(height - 0.55, 0.0);
    double stage_two_height = Math.max(stage_one_height - 0.66, 0.0);
    // Logger.recordOutput(
    // "testingPose", new Pose3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0.0, 0.0, 0.0)));
    Logger.recordOutput(
        "componentPositions",
        new Pose3d[] {
          new Pose3d(new Translation3d(0.05, 0.01, 0.9), new Rotation3d(0.0, ramp_rotation, 0.0)),
          new Pose3d(
              new Translation3d(-0.16, 0.31, 0.115), new Rotation3d(climb_rotation, 0.0, 0.0)),
          new Pose3d(
              new Translation3d(0.34, 0.12, height + 0.35),
              new Rotation3d(0.0, -claw_rotation + 0.465719787 * 180.0, 0.0)),
          new Pose3d(new Translation3d(0.0, 0.0, height), new Rotation3d(0.0, 0.0, 0.0)),
          new Pose3d(new Translation3d(0.0, 0.0, stage_two_height), new Rotation3d(0.0, 0.0, 0.0)),
          new Pose3d(new Translation3d(0.0, 0.0, stage_one_height), new Rotation3d(0.0, 0.0, 0.0))
        });
  }

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
    ClimbConstants.synced.loadData();
  }

  public void configureAutos() {
    boolean firstDefault = false;
    File autoDirectory =
        new File(Filesystem.getDeployDirectory().toPath().resolve("auto").toString());
    strategyContainer = new AutoStrategyContainer(autoDirectory.listFiles());
    for (AutoStrategy strategy : strategyContainer.getStrategies()) {
      if (strategy.autoStrategyName.equals("1PieceBarge")) {
        autoChooser.setDefaultOption(strategy.autoStrategyName, strategy);
        firstDefault = true;
      } else {
        autoChooser.addOption(strategy.autoStrategyName, strategy);
      }
    }

    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  public void configureSubsystems() {
    if (FeatureFlags.synced.getObject().runDrive) {
      drive = InitSubsystems.initDriveSubsystem();
      if (FeatureFlags.synced.getObject().runVision) {
        vision = InitSubsystems.initVisionSubsystem(drive);
        drive.setAlignmentSupplier(vision::getDistanceErrorToTag);
      }
    }
    if (FeatureFlags.synced.getObject().runRamp) {
      rampSubsystem = InitSubsystems.initRampSubsystem();
    }
    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem = InitSubsystems.initClimbSubsystem();
      if (FeatureFlags.synced.getObject().runRamp) {
        climbSubsystem.setRampClear(() -> rampSubsystem.isInPosition());
      }
    }

    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem = InitSubsystems.initScoringSubsystem();
      if (FeatureFlags.synced.getObject().runDrive) {
        scoringSubsystem.setIsDriveLinedUpSupplier(
            () -> {
              switch (strategyManager.getAutonomyMode()) {
                case Manual:
                  return InitBindings.isManualScorePressed();
                case Smart:
                case Mixed:
                  return drive.isDriveAlignmentFinished() || InitBindings.isManualScorePressed();
                case Full:
                default:
                  if (scoringSubsystem.getGamePiece() == GamePiece.Algae) {
                    return drive.isDriveCloseToFinalLineupPose();
                  } else {
                    return drive.isDriveAlignmentFinished();
                  }
              }
            });
        scoringSubsystem.setReefDistanceSupplier(
            () -> {
              Translation2d reefCenter =
                  drive.isAllianceRed()
                      ? JsonConstants.redFieldLocations.redReefCenterTranslation
                      : JsonConstants.blueFieldLocations.blueReefCenterTranslation;
              return Meters.of(drive.getPose().getTranslation().getDistance(reefCenter));
            });
      } else {
        scoringSubsystem.setIsDriveLinedUpSupplier(() -> true);
      }
    }

    if (FeatureFlags.synced.getObject().runLEDs) {
      led = InitSubsystems.initLEDs(scoringSubsystem, climbSubsystem, drive);
      if (FeatureFlags.synced.getObject().runVision) {
        // led.setVisionWorkingSupplier(() -> vision.coprocessorConnected());
      } else {
        // led.setVisionWorkingSupplier(() -> false);
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
    InitBindings.initControllers();
    if (FeatureFlags.synced.getObject().runDrive) {
      InitBindings.initDriveBindings(drive, strategyManager);
    }
    if (FeatureFlags.synced.getObject().runRamp) {
      InitBindings.initRampBindings(rampSubsystem);
    }
    if (FeatureFlags.synced.getObject().runClimb) {
      InitBindings.initClimbBindings(climbSubsystem);
    }
    if (FeatureFlags.synced.getObject().runScoring) {
      InitBindings.initScoringBindings(scoringSubsystem);
    }
  }

  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return AkitDriveCommands.feedforwardCharacterization(drive);
  }

  public void periodic() {

    strategyManager.periodic();

    Logger.recordOutput("Switches/brake", brakeSwitch.get());
    Logger.recordOutput("Switches/led", ledSwitch.get());
  }

  public void autonomousInit() {
    drive.autonomousInit();

    // setSubsystemsToBrake();
    // if (FeatureFlags.synced.getObject().runLEDs) {
    //   led.setLedOn(true);
    // }

    // load chosen strategy
    strategyManager.autonomousInit(autoChooser.getSelected());
  }

  public void teleopInit() {
    if (FeatureFlags.synced.getObject().runDrive) {
      drive.teleopInit();
    }

    setSubsystemsToBrake();
    if (FeatureFlags.synced.getObject().runLEDs) {
      led.setLedOn(true);
    }

    strategyManager.teleopInit();
  }

  /** This method must be called from robot, as it isn't called automatically */
  public void testInit() {
    InitBindings.initTestModeBindings();

    switch (TestModeManager.getTestMode()) {
      case ElevatorCharacterization:
        CommandScheduler.getInstance()
            .schedule(
                new SequentialCommandGroup(
                    new WaitCommand(2.0),
                    new TuneS(
                        scoringSubsystem.getElevatorMechanismForTuning(),
                        RotationsPerSecond.of(0.001),
                        0.1)));
        break;
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

      case LEDTest:
        // CommandScheduler.getInstance().schedule(led.runCycle());
      default:
        break;
    }
  }

  /** This method must be called from the robot, as it isn't called automatically. */
  public void testPeriodic() {
    if (FeatureFlags.synced.getObject().runScoring) {
      scoringSubsystem.testPeriodic();
    }

    if (FeatureFlags.synced.getObject().runRamp) {
      rampSubsystem.testPeriodic();
    }

    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem.testPeriodic();
    }

    if (FeatureFlags.synced.getObject().runDrive) {
      drive.periodic();
    }
  }

  public void disabledPeriodic() {
    led.periodic();
    // Logger.recordOutput("feature_flags/drive", FeatureFlags.synced.getObject().runDrive);
    strategyManager.logActions();
    checkSwitchForDisabled();
    checkLedSwitch();

    if (drive != null) {
      drive.disabledPeriodic();
    }

    Logger.recordOutput("Switches/brake", brakeSwitch.get());
    Logger.recordOutput("Switches/led", ledSwitch.get());
  }

  public void disabledInit() {
    CommandScheduler.getInstance().cancelAll();

    if (FeatureFlags.synced.getObject().runClimb) {
      climbSubsystem.setBrakeMode(true);
    }
  }

  public void updateMapleSim() {
    SimulatedArena.getInstance().simulationPeriodic();
    if (driveSim != null) {
      Logger.recordOutput(
          "FieldSimulation/RobotPosition", RobotContainer.driveSim.getSimulatedDriveTrainPose());
    }
    Logger.recordOutput(
        "FieldSimulation/Coral", SimulatedArena.getInstance().getGamePiecesArrayByType("Coral"));
    Logger.recordOutput(
        "FieldSimulation/Algae", SimulatedArena.getInstance().getGamePiecesArrayByType("Algae"));
  }
}
