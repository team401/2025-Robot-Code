package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import coppercore.controls.state_machine.StateMachine;
import coppercore.controls.state_machine.StateMachineConfiguration;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.state.StateContainer;
import coppercore.vision.VisionLocalizer.DistanceToTag;
import coppercore.wpilib_interface.DriveTemplate;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.subsystems.drive.states.IdleState;
import frc.robot.subsystems.drive.states.JoystickDrive;
import frc.robot.subsystems.drive.states.LineupState;
import frc.robot.subsystems.drive.states.OTFState;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.util.LocalADStarAK;
import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.BooleanSupplier;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive implements DriveTemplate {
  // JsonConstants.tunerConstants doesn't include these constants, so they are declared locally
  static final double ODOMETRY_FREQUENCY =
      new CANBus(JsonConstants.drivetrainConstants.DrivetrainConstants.CANBusName).isNetworkFD()
          ? 250.0
          : 100.0;
  public static final double DRIVE_BASE_RADIUS =
      Math.max(
          Math.max(
              Math.hypot(
                  DriveConfiguration.getInstance().FrontLeft.LocationX,
                  DriveConfiguration.getInstance().FrontLeft.LocationY),
              Math.hypot(
                  DriveConfiguration.getInstance().FrontRight.LocationX,
                  DriveConfiguration.getInstance().FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  DriveConfiguration.getInstance().BackLeft.LocationX,
                  DriveConfiguration.getInstance().BackLeft.LocationY),
              Math.hypot(
                  DriveConfiguration.getInstance().BackRight.LocationX,
                  DriveConfiguration.getInstance().BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              DriveConfiguration.getInstance().FrontLeft.WheelRadius,
              JsonConstants.drivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(DriveConfiguration.getInstance().FrontLeft.DriveMotorGearRatio),
              DriveConfiguration.getInstance().FrontLeft.SlipCurrent,
              1),
          getModuleTranslations());

  static final Lock odometryLock = new ReentrantLock();
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR
  private final SysIdRoutine sysId;
  private final Alert gyroDisconnectedAlert =
      new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
  private Rotation2d rawGyroRotation = new Rotation2d();
  private SwerveModulePosition[] lastModulePositions = // For delta tracking
      new SwerveModulePosition[] {
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition(),
        new SwerveModulePosition()
      };
  private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

  private ChassisSpeeds goalSpeeds = new ChassisSpeeds();

  public ProfiledPIDController angleController =
      new ProfiledPIDController(
          JsonConstants.drivetrainConstants.rotationAlignKp,
          JsonConstants.drivetrainConstants.rotationAlignKi,
          JsonConstants.drivetrainConstants.rotationAlignKd,
          new TrapezoidProfile.Constraints(
              JsonConstants.drivetrainConstants.maxRotationAlignVelocity,
              JsonConstants.drivetrainConstants.maxRotationAlignAcceleration));

  public enum DesiredLocation {
    Reef0,
    Reef1,
    Reef2,
    Reef3,
    Reef4,
    Reef5,
    Reef6,
    Reef7,
    Reef8,
    Reef9,
    Reef10,
    Reef11,
    Algae0,
    Algae1,
    Algae2,
    Algae3,
    Algae4,
    Algae5,
    Processor,
    CoralStationLeft,
    CoralStationRight,
  }

  public static final DesiredLocation[] reefCoralLocations = {
    DesiredLocation.Reef0,
    DesiredLocation.Reef1,
    DesiredLocation.Reef2,
    DesiredLocation.Reef3,
    DesiredLocation.Reef4,
    DesiredLocation.Reef5,
    DesiredLocation.Reef6,
    DesiredLocation.Reef7,
    DesiredLocation.Reef8,
    DesiredLocation.Reef9,
    DesiredLocation.Reef10,
    DesiredLocation.Reef11,
  };

  public DesiredLocation[] locationArray = {
    DesiredLocation.Reef0,
    DesiredLocation.Reef1,
    DesiredLocation.Reef2,
    DesiredLocation.Reef3,
    DesiredLocation.Reef4,
    DesiredLocation.Reef5,
    DesiredLocation.Reef6,
    DesiredLocation.Reef7,
    DesiredLocation.Reef8,
    DesiredLocation.Reef9,
    DesiredLocation.Reef10,
    DesiredLocation.Reef11,
    DesiredLocation.Processor,
    DesiredLocation.CoralStationLeft,
    DesiredLocation.CoralStationRight
  };

  public static final DesiredLocation[] reefAlgaeLocations = {
    DesiredLocation.Algae0,
    DesiredLocation.Algae1,
    DesiredLocation.Algae2,
    DesiredLocation.Algae3,
    DesiredLocation.Algae4,
    DesiredLocation.Algae5
  };

  private DesiredLocation desiredLocation = DesiredLocation.Reef9;
  private DesiredLocation intakeLocation = DesiredLocation.CoralStationLeft;
  private boolean goToIntake = false;

  @AutoLogOutput(key = "Drive/waitOnScore")
  private BooleanSupplier waitOnScore = () -> false;

  @AutoLogOutput(key = "Drive/waitOnIntake")
  private BooleanSupplier waitOnIntake = () -> false;

  private VisionAlignment alignmentSupplier = null;

  private static Drive instance;

  private boolean driveLinedUp = false;

  private enum DriveState implements StateContainer {
    Idle(new IdleState(instance)),
    OTF(new OTFState(instance)),
    Lineup(new LineupState(instance)),
    Joystick(new JoystickDrive(instance));
    private final PeriodicStateInterface state;

    DriveState(PeriodicStateInterface state) {
      this.state = state;
    }

    @Override
    public PeriodicStateInterface getState() {
      return state;
    }
  }

  public enum DriveTrigger {
    ManualJoysticks,
    CancelAutoAlignment,
    FinishOTF,
    CancelOTF,
    BeginOTF,
    BeginLineup,
    CancelLineup,
    FinishLineup,
    WaitForScore,
  }

  private StateMachineConfiguration<DriveState, DriveTrigger> stateMachineConfiguration;

  private StateMachine<DriveState, DriveTrigger> stateMachine;

  private boolean isAligningToFieldElement = false;
  private Translation2d lockedAlignPosition = new Translation2d();

  private LocalADStarAK localADStar = new LocalADStarAK();

  private Command warmupCommand = PathfindingCommand.warmupCommand();

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, DriveConfiguration.getInstance().FrontLeft);
    modules[1] = new Module(frModuleIO, 1, DriveConfiguration.getInstance().FrontRight);
    modules[2] = new Module(blModuleIO, 2, DriveConfiguration.getInstance().BackLeft);
    modules[3] = new Module(brModuleIO, 3, DriveConfiguration.getInstance().BackRight);

    // Usage reporting for swerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

    // Start odometry thread
    PhoenixOdometryThread.getInstance().start();

    // Configure AutoBuilder for PathPlanner
    AutoBuilder.configure(
        this::getPose,
        this::setPose,
        this::getChassisSpeeds,
        (ChassisSpeeds speeds) -> this.setGoalSpeeds(speeds, false),
        new PPHolonomicDriveController(
            new PIDConstants(1.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(localADStar);
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });
    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });

    // warm up java processing for faster pathfind later
    if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
      localADStar.setDynamicObstacles(
          List.of(
              new Pair<Translation2d, Translation2d>(
                  JsonConstants.redFieldLocations.coralAlgaeStackLeftTopCorner,
                  JsonConstants.redFieldLocations.coralAlgaeStackLeftBottomCorner),
              new Pair<Translation2d, Translation2d>(
                  JsonConstants.redFieldLocations.coralAlgaeStackMiddleTopCorner,
                  JsonConstants.redFieldLocations.coralAlgaeStackMiddleBottomCorner),
              new Pair<Translation2d, Translation2d>(
                  JsonConstants.redFieldLocations.coralAlgaeStackRightTopCorner,
                  JsonConstants.redFieldLocations.coralAlgaeStackRightBottomCorner)),
          getPose().getTranslation());
    } else {
      localADStar.setDynamicObstacles(
          List.of(
              new Pair<Translation2d, Translation2d>(
                  JsonConstants.blueFieldLocations.coralAlgaeStackLeftTopCorner,
                  JsonConstants.blueFieldLocations.coralAlgaeStackLeftBottomCorner),
              new Pair<Translation2d, Translation2d>(
                  JsonConstants.blueFieldLocations.coralAlgaeStackMiddleTopCorner,
                  JsonConstants.blueFieldLocations.coralAlgaeStackMiddleBottomCorner),
              new Pair<Translation2d, Translation2d>(
                  JsonConstants.blueFieldLocations.coralAlgaeStackRightTopCorner,
                  JsonConstants.blueFieldLocations.coralAlgaeStackRightBottomCorner)),
          getPose().getTranslation());
    }

    warmupCommand.schedule();

    angleController.enableContinuousInput(-Math.PI, Math.PI);
    // Configure SysId
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));

    configureStates();
  }

  public void configureStates() {
    instance = this;

    stateMachineConfiguration = new StateMachineConfiguration<>();

    stateMachineConfiguration
        .configure(DriveState.Joystick)
        .permit(DriveTrigger.BeginOTF, DriveState.OTF);

    stateMachineConfiguration
        .configure(DriveState.OTF)
        .permit(DriveTrigger.CancelOTF, DriveState.Joystick)
        .permitIf(
            DriveTrigger.FinishOTF,
            DriveState.Joystick,
            () -> this.isDriveCloseToFinalLineupPose() && !this.isDesiredLocationReef())
        .permitIf(
            DriveTrigger.FinishOTF,
            DriveState.Lineup,
            () -> (this.isDriveCloseToFinalLineupPose() && this.isDesiredLocationReef()))
        .permit(DriveTrigger.CancelAutoAlignment, DriveState.Joystick)
        .permitIf(DriveTrigger.BeginLineup, DriveState.Lineup, () -> this.isDesiredLocationReef());

    stateMachineConfiguration
        .configure(DriveState.Lineup)
        .permit(DriveTrigger.CancelLineup, DriveState.Joystick)
        .permit(DriveTrigger.CancelAutoAlignment, DriveState.Joystick)
        .permit(DriveTrigger.BeginOTF, DriveState.OTF);

    stateMachine = new StateMachine<>(stateMachineConfiguration, DriveState.Joystick);
  }

  public void autonomousInit() {

    if (warmupCommand != null && warmupCommand.isScheduled()) {
      warmupCommand.cancel();
    }
  }

  /** remove algae coral stack obstacles for on the fly */
  public void teleopInit() {
    localADStar.setDynamicObstacles(List.of(), getPose().getTranslation());
  }

  @Override
  public void periodic() {
    // Manually cancel go to intake if we have a gamepiece
    if (goToIntake && ScoringSubsystem.getInstance().isCoralDetected()) {
      setGoToIntake(false);
    } else if (goToIntake && ScoringSubsystem.getInstance().isAlgaeDetected()) {
      setGoToIntake(false);
    }

    odometryLock.lock(); // Prevents odometry updates while reading data
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }
    odometryLock.unlock();

    // Stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }
    }

    stateMachine.periodic();
    Logger.recordOutput("Drive/State", stateMachine.getCurrentState());

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // allow controlling drive unless we are characterizing
    if (DriverStation.isTestEnabled()) {
      switch (TestModeManager.getTestMode()) {
        case DriveFeedForwardCharacterization:
        case DriveSteerMotorCharacterization:
          break;
        default:
          this.runVelocity();
      }
    }
    // run velocity if not disabled
    else if (!DriverStation.isDisabled()) {
      this.runVelocity();
    }

    // Update odometry
    double[] sampleTimestamps =
        modules[0].getOdometryTimestamps(); // All signals are sampled together
    int sampleCount = sampleTimestamps.length;
    for (int i = 0; i < sampleCount; i++) {
      // Read wheel positions and deltas from each module
      SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
      SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
      for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
        modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
        moduleDeltas[moduleIndex] =
            new SwerveModulePosition(
                modulePositions[moduleIndex].distanceMeters
                    - lastModulePositions[moduleIndex].distanceMeters,
                modulePositions[moduleIndex].angle);
        lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
      }

      // Update gyro angle
      if (gyroInputs.connected) {
        // Use the real gyro angle
        rawGyroRotation = gyroInputs.odometryYawPositions[i];
      } else {
        // Use the angle delta from the kinematics and module deltas
        Twist2d twist = kinematics.toTwist2d(moduleDeltas);
        rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
      }

      // Apply update
      poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
    }

    // Update gyro alert
    gyroDisconnectedAlert.set(
        !gyroInputs.connected && ModeConstants.currentMode == ModeConstants.Mode.REAL);

    Logger.recordOutput("Drive/goToIntake", goToIntake);
    Logger.recordOutput("Drive/driveLinedUp", driveLinedUp);
  }

  public void disabledPeriodic() {
    if (warmupCommand != null) {
      Logger.recordOutput("Drive/warmupScheduled", warmupCommand.isScheduled());
    }
  }

  /**
   * sets the location to the reef pole directly next to current NOTE: this will not change sides of
   * reef (stays on one side of hexagon)
   */
  public void sidestepReefLocation() {
    int reefIndex = getDesiredLocationIndex();
    if (reefIndex > 11 || reefIndex < 0) {
      // not a reef location
      return;
    }

    if (reefIndex % 2 == 0) {
      desiredLocation = locationArray[reefIndex + 1];
    } else {
      desiredLocation = locationArray[reefIndex - 1];
    }
  }

  /**
   * sets desired speeds of robot
   *
   * @param speeds - desired speeds of robot
   * @param fieldCentric - true if controlling in teleop (allows driving with field-oriented
   *     control), false for auto (robot centric)
   */
  public void setGoalSpeeds(ChassisSpeeds speeds, boolean fieldCentric) {
    if (fieldCentric && stateMachine.inState(DriveState.Joystick)) {
      // Adjust for field-centric control
      boolean isFlipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      Rotation2d robotRotation =
          isFlipped
              ? getRotation().plus(new Rotation2d(Math.PI)) // Flip orientation for Red Alliance
              : getRotation();

      this.goalSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, robotRotation);
    } else {
      Logger.recordOutput("Drive/DesiredRobotCentricSpeeds", speeds);
      this.goalSpeeds = speeds;
    }
  }

  public void alignToFieldElement() {
    if (isDesiredLocationReef()) {
      lockedAlignPosition =
          isAllianceRed()
              ? JsonConstants.redFieldLocations.redReefCenterTranslation
              : JsonConstants.blueFieldLocations.blueReefCenterTranslation;
      isAligningToFieldElement = true;
    }
  }

  public void disableAlign() {
    isAligningToFieldElement = false;
  }

  public boolean isAligningToFieldElement() {
    return isAligningToFieldElement;
  }

  /**
   * set supplier that interfaces with scoring used to make drive wait until setting next location
   * in auto
   *
   * @param waitOnScore BooleanSupplier to let drive know if scoring is done scoring
   */
  public void setWaitOnScoreSupplier(BooleanSupplier waitOnScore) {
    this.waitOnScore = waitOnScore;
  }

  /**
   * set supplier that interfaces with intake to make drive wait until setting next location in auto
   *
   * @param waitOnIntake BooleanSupplier to let drive know if intake has a coral for auto
   */
  public void setWaitOnIntakeSupplier(BooleanSupplier waitOnIntake) {
    this.waitOnIntake = waitOnIntake;
  }

  /**
   * sets the supplier for landing zone alignment help
   *
   * @param alignmentSupplier from vision.getDistanceErrorToTag; helps drive align to reef
   */
  public void setAlignmentSupplier(VisionAlignment alignmentSupplier) {
    this.alignmentSupplier = alignmentSupplier;
  }

  /**
   * gets the alignment supplier for use in lineup state
   *
   * @return VisionAlignment supplier (possibly null)
   */
  public VisionAlignment getVisionAlignment() {
    return alignmentSupplier;
  }

  /**
   * checks if drive needs to wait on scoring subsystem
   *
   * @return true if drive is waiting
   */
  public boolean isWaitingOnScore() {
    return waitOnScore.getAsBoolean();
  }

  /**
   * checks if drive is currently following an on the fly path
   *
   * @return state of OTF following
   */
  @AutoLogOutput(key = "Drive/OTF/isOTF")
  public boolean isDriveOTF() {
    return stateMachine.inState(DriveState.OTF);
  }

  /**
   * Get the current instance of the drive subsystem
   *
   * <p>This should be used for scoring to warmup otf while scoring
   *
   * @return Current instance of the drive subsystem.
   */
  public static Drive getInstance() {
    return instance;
  }

  /**
   * checks if robot pose is sufficiently close to desired pose
   *
   * @return true if robot pose is close to desired pose
   */
  @AutoLogOutput(key = "Drive/OTF/isDriveCloseToFinalLineupPose")
  public boolean isDriveCloseToFinalLineupPose() {
    // relative to transforms first pose into distance from desired pose
    // then get distance between poses (if less than 0.1 meters we are good)
    Logger.recordOutput(
        "Drive/distanceToLineupNewMethod",
        this.getPose()
            .getTranslation()
            .getDistance(OTFState.findOTFPoseFromDesiredLocation(this).getTranslation()));
    return this.getPose()
            .getTranslation()
            .getDistance(OTFState.findOTFPoseFromDesiredLocation(this).getTranslation())
        < JsonConstants.drivetrainConstants.otfPoseDistanceLimit;
  }

  @AutoLogOutput(key = "Drive/OTF/isDriveCloseForFarWarmup")
  public boolean isDriveCloseForFarWarmup() {
    return this.getPose()
            .getTranslation()
            .getDistance(OTFState.findOTFPoseFromDesiredLocation(this).getTranslation())
        < JsonConstants.drivetrainConstants.otfFarWarmupDistance;
  }

  public boolean isDriveCloseForWarmup() {
    return this.getPose()
            .getTranslation()
            .getDistance(OTFState.findOTFPoseFromDesiredLocation(this).getTranslation())
        < JsonConstants.drivetrainConstants.otfWarmupDistance;
  }

  /**
   * checks if drive is currently lining up to a reef
   *
   * @return state of lining up
   */
  @AutoLogOutput(key = "Drive/Lineup/isLiningUp")
  public boolean isDriveLiningUp() {
    return stateMachine.inState(DriveState.Lineup);
  }

  /**
   * checks if desired locaiton is set to a reef location
   *
   * @return true if location is reef; false otherwise (processor / coral station)
   */
  public boolean isDesiredLocationReef() {
    boolean isCoralReefTarget =
        !(desiredLocation == DesiredLocation.CoralStationLeft
                || desiredLocation == DesiredLocation.CoralStationRight
                || desiredLocation == DesiredLocation.Processor)
            && !goToIntake;
    boolean isAlgaeReefTarget = isLocationAlgaeIntake(intakeLocation) && goToIntake;
    return isCoralReefTarget || isAlgaeReefTarget;
  }

  /**
   * checks if location is a scoring location
   *
   * @return true if location is scoring (reef / processor)
   */
  @AutoLogOutput(key = "Drive/isLocationScoring")
  public boolean isLocationScoring(DesiredLocation location) {
    boolean isLocationCoralStation =
        (location == DesiredLocation.CoralStationLeft
            || location == DesiredLocation.CoralStationRight);
    boolean isAlgaeIntake = isLocationAlgaeIntake(location);

    return (!isLocationCoralStation && !isAlgaeIntake);
  }

  /** checks if location is reef (center of poles) */
  public boolean isLocationAlgaeIntake(DesiredLocation location) {
    return (location == DesiredLocation.Algae0
        || location == DesiredLocation.Algae1
        || location == DesiredLocation.Algae2
        || location == DesiredLocation.Algae3
        || location == DesiredLocation.Algae4
        || location == DesiredLocation.Algae5);
  }

  /**
   * sets desired path location calling this and then setting OTF to true will cause robot to drive
   * path from current pose to the location
   *
   * @param location desired location for robot to pathfind to
   */
  public void setDesiredLocation(DesiredLocation location) {
    if (isLocationScoring(location)) {
      this.desiredLocation = location;
    }
  }

  /**
   * @return the desired location for otf and lineup
   */
  @AutoLogOutput(key = "Drive//DesiredLocation")
  public DesiredLocation getDesiredLocation() {
    return goToIntake ? this.intakeLocation : this.desiredLocation;
  }

  /**
   * get intake location currently set
   *
   * @return the intake location currently set
   */
  public DesiredLocation getDesiredIntakeLocation() {
    return this.intakeLocation;
  }

  /**
   * returns index of reef location for interfacing with snakescreen
   *
   * @return a double representing the index of reef location
   */
  public int getDesiredLocationIndex() {
    for (int i = 0; i < locationArray.length; i++) {
      if (locationArray[i] == desiredLocation) {
        return i;
      }
    }
    return -1;
  }

  /**
   * returns index of reef location for interfacing with snakescreen
   *
   * @return a double representing the index of reef location
   */
  public int getDesiredAlgaeLocationIndex() {
    for (int i = 0; i < reefAlgaeLocations.length; i++) {
      if (reefAlgaeLocations[i] == desiredLocation) {
        return i;
      }
    }
    return -1;
  }

  /**
   * sets desired path location calling this and then setting OTF to true will cause robot to drive
   * path from current pose to the location
   *
   * @param locationIndex desired location index from DesiredLocationSelector
   */
  public void setDesiredLocation(int locationIndex) {
    this.desiredLocation = locationArray[locationIndex];
  }

  /** checks for update from reef location network table (SnakeScreen) run periodically in drive */
  public void updateDesiredLocationFromNetworkTables(double desiredIndex, boolean isAlgae) {
    if (desiredIndex == -1) {
      return;
    }

    if (isAlgae && reefAlgaeLocations[(int) desiredIndex] != intakeLocation) {
      this.setDesiredIntakeLocation(reefAlgaeLocations[(int) desiredIndex]);
      if (isDriveOTF()) {
        this.fireTrigger(DriveTrigger.ManualJoysticks);
        this.fireTrigger(DriveTrigger.BeginOTF);
      }
      return;
    } else if (!isAlgae && locationArray[(int) desiredIndex] != desiredLocation) {
      if (isDriveOTF()) {
        this.updateDesiredLocation((int) desiredIndex);
      } else {
        this.setDesiredLocation((int) desiredIndex);
      }
    }
  }

  /**
   * updates desired path location (for when OTF is already running) this will cancel old command
   * and generate a new OTF path to run
   *
   * @param locationIndex desired location index for robot to pathfind to (sent from
   *     DesiredLocationSelector)
   */
  public void updateDesiredLocation(int locationIndex) {
    this.updateDesiredLocation(locationArray[locationIndex]);
  }

  /** sets brake mode for each module */
  public void setBrakeMode(boolean brake) {
    for (int i = 0; i < modules.length; i++) {
      modules[i].setBrakeMode(brake);
    }
  }

  /**
   * updates desired path location (for when OTF is already running) this will cancel old command
   * and generate a new OTF path to run
   *
   * @param location desired location for robot to pathfind to
   */
  public void updateDesiredLocation(DesiredLocation location) {
    this.setDesiredLocation(location);

    if (isDriveOTF()) {
      stateMachine.fire(DriveTrigger.ManualJoysticks);
      stateMachine.fire(DriveTrigger.BeginOTF);
    }
  }

  /**
   * set which intake to go to
   *
   * @param location coral station location (left v right)
   */
  public void setDesiredIntakeLocation(DesiredLocation location) {
    if (!isLocationScoring(location)) {
      this.intakeLocation = location;
    }
  }

  /**
   * tells drive to otf to intake instead of reef
   *
   * @param goToIntake true if drive should go to coral stations
   */
  public void setGoToIntake(boolean goToIntake) {
    this.goToIntake = goToIntake;
  }

  /**
   * checks if drive is in intake mode
   *
   * @return true if drive is going to a coral station
   */
  @AutoLogOutput(key = "Drive/goToIntake")
  public boolean isGoingToIntake() {
    return goToIntake;
  }

  /**
   * attempts to change state of state machine
   *
   * @param trigger trigger to give to state for transition
   */
  public void fireTrigger(DriveTrigger trigger) {
    stateMachine.fire(trigger);
  }

  public void setDriveLinedUp(boolean linedUp) {
    this.driveLinedUp = linedUp;
  }

  public boolean isDriveLineupFinished() {
    return driveLinedUp;
  }

  /**
   * checks if alignment has run
   *
   * @return true if we are close to otf for intake OR we have finished lineup for reef
   */
  public boolean isDriveAlignmentFinished() {
    return goToIntake ? this.isDriveCloseToFinalLineupPose() : this.driveLinedUp;
  }

  /**
   * checks if driver station alliance is red
   *
   * @return true if alliance is red
   */
  public boolean isAllianceRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  public void alignToTarget() {
    double omega = 0.0;

    // Get current position
    Translation2d currentPosition = getPose().getTranslation();

    // Calculate desired angle to face the target position
    double targetAngle =
        Math.atan2(
            lockedAlignPosition.getY() - currentPosition.getY(),
            lockedAlignPosition.getX() - currentPosition.getX());

    Logger.recordOutput("Drive/targetAngle", targetAngle);

    // Use PID to rotate toward the target angle
    omega = angleController.calculate(getRotation().getRadians(), targetAngle);
    setGoalSpeeds(
        new ChassisSpeeds(goalSpeeds.vxMetersPerSecond, goalSpeeds.vyMetersPerSecond, omega),
        false);
  }

  /** Runs the drive at the desired speeds set in (@Link setGoalSpeeds) */
  public void runVelocity() {
    if (isAligningToFieldElement) {
      alignToTarget();
    }

    // Calculate module setpoints
    ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(this.goalSpeeds, 0.02);
    SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(
        setpointStates, JsonConstants.drivetrainConstants.kSpeedAt12Volts);

    // Log unoptimized setpoints and setpoint speeds
    Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

    // Send setpoints to modules
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(setpointStates[i]);
    }

    // Log optimized setpoints (runSetpoint mutates each state)
    Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    Logger.recordOutput("desired_location", desiredLocation);
  }

  /** Runs the drive in a straight line with the specified drive output. */
  public void runCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runCharacterization(output);
    }
  }

  /** Runs one steer motor with the specified turn output */
  public void runSteerCharacterization(double output) {
    for (int i = 0; i < 4; i++) {
      modules[i].runSteerCharacterization(output);
    }
  }

  /** Stops the drive. */
  public void stop() {
    setGoalSpeeds(new ChassisSpeeds(), false);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    Rotation2d[] headings = new Rotation2d[4];
    for (int i = 0; i < 4; i++) {
      headings[i] = getModuleTranslations()[i].getAngle();
    }
    kinematics.resetHeadings(headings);
    stop();
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0))
        .withTimeout(1.0)
        .andThen(sysId.quasistatic(direction));
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return run(() -> runCharacterization(0.0)).withTimeout(1.0).andThen(sysId.dynamic(direction));
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules. */
  @AutoLogOutput(key = "SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getState();
    }
    return states;
  }

  /** Returns the module positions (turn angles and drive positions) for all of the modules. */
  private SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] states = new SwerveModulePosition[4];
    for (int i = 0; i < 4; i++) {
      states[i] = modules[i].getPosition();
    }
    return states;
  }

  /** Returns the measured chassis speeds of the robot. */
  @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  /** Returns the position of each module in radians. */
  public double[] getWheelRadiusCharacterizationPositions() {
    double[] values = new double[4];
    for (int i = 0; i < 4; i++) {
      values[i] = modules[i].getWheelRadiusCharacterizationPosition();
    }
    return values;
  }

  /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
  public double getFFCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getFFCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  public double getSteerCharacterizationVelocity() {
    double output = 0.0;
    for (int i = 0; i < 4; i++) {
      output += modules[i].getSteerCharacterizationVelocity() / 4.0;
    }
    return output;
  }

  /** Returns the current odometry pose. */
  @AutoLogOutput(key = "Odometry/Robot")
  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  /** Returns the current odometry rotation. */
  public Rotation2d getRotation() {
    return getPose().getRotation();
  }

  /** Resets the current odometry pose. */
  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
  }

  /** Adds a new timestamped vision measurement. */
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    poseEstimator.addVisionMeasurement(
        visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
  }

  /** Returns the maximum linear speed in meters per sec. */
  public double getMaxLinearSpeedMetersPerSec() {
    return JsonConstants.drivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  /** Returns the maximum angular speed in radians per sec. */
  public double getMaxAngularSpeedRadPerSec() {
    return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
  }

  /** Returns an array of module translations. */
  public static Translation2d[] getModuleTranslations() {
    return new Translation2d[] {
      new Translation2d(
          DriveConfiguration.getInstance().FrontLeft.LocationX,
          DriveConfiguration.getInstance().FrontLeft.LocationY),
      new Translation2d(
          DriveConfiguration.getInstance().FrontRight.LocationX,
          DriveConfiguration.getInstance().FrontRight.LocationY),
      new Translation2d(
          DriveConfiguration.getInstance().BackLeft.LocationX,
          DriveConfiguration.getInstance().BackLeft.LocationY),
      new Translation2d(
          DriveConfiguration.getInstance().BackRight.LocationX,
          DriveConfiguration.getInstance().BackRight.LocationY)
    };
  }

  @FunctionalInterface
  public static interface VisionAlignment {
    public DistanceToTag get(
        int tagId,
        int desiredCameraIndex,
        double crossTrackOffsetMeters,
        double alongTrackOffsetMeters);
  }
}
