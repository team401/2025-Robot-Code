package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.vision.VisionLocalizer.DistanceToTag;
import coppercore.wpilib_interface.DriveTemplate;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
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
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.util.LocalADStarAK;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
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
    Processor,
    CoralStationLeft,
    CoralStationRight,
  }

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

  private DesiredLocation desiredLocation = DesiredLocation.Reef0;

  private final int FrontLeftCameraIndex = 0;
  private final int FrontRightCameraIndex = 1;

  private boolean isOTF = false;

  private boolean isLiningUp = false;

  private Command driveToPose = null;

  private VisionAlignment alignmentSupplier;

  // along track pid test mode
  private LoggedTunableNumber alongTrackkP =
      new LoggedTunableNumber(
          "DriveLineupGains/AlongTrackkP", JsonConstants.drivetrainConstants.driveAlongTrackkP);
  private LoggedTunableNumber alongTrackkI =
      new LoggedTunableNumber(
          "DriveLineupGains/AlongTrackkI", JsonConstants.drivetrainConstants.driveAlongTrackkI);
  private LoggedTunableNumber alongTrackkD =
      new LoggedTunableNumber(
          "DriveLineupGains/AlongTrackkD", JsonConstants.drivetrainConstants.driveAlongTrackkD);

  // cross tack pid test mode
  private LoggedTunableNumber crossTrackkP =
      new LoggedTunableNumber(
          "DriveLineupGains/CrossTrackkP", JsonConstants.drivetrainConstants.driveCrossTrackkP);
  private LoggedTunableNumber crossTrackkI =
      new LoggedTunableNumber(
          "DriveLineupGains/CrossTrackkI", JsonConstants.drivetrainConstants.driveCrossTrackkI);
  private LoggedTunableNumber crossTrackkD =
      new LoggedTunableNumber(
          "DriveLineupGains/CrossTrackkD", JsonConstants.drivetrainConstants.driveCrossTrackkD);

  // rotation pid test mode
  private LoggedTunableNumber rotationkP =
      new LoggedTunableNumber(
          "DriveLineupGains/rotationkP", JsonConstants.drivetrainConstants.driveRotationkP);
  private LoggedTunableNumber rotationkI =
      new LoggedTunableNumber(
          "DriveLineupGains/rotationkI", JsonConstants.drivetrainConstants.driveRotationkI);
  private LoggedTunableNumber rotationkD =
      new LoggedTunableNumber(
          "DriveLineupGains/rotationkP", JsonConstants.drivetrainConstants.driveRotationkD);

  private PIDController driveAlongTrackLineupController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveAlongTrackkP,
          JsonConstants.drivetrainConstants.driveAlongTrackkI,
          JsonConstants.drivetrainConstants.driveAlongTrackkD);
  private PIDController driveCrossTrackLineupController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveCrossTrackkP,
          JsonConstants.drivetrainConstants.driveCrossTrackkI,
          JsonConstants.drivetrainConstants.driveCrossTrackkD);
  private PIDController rotationController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveRotationkP,
          JsonConstants.drivetrainConstants.driveRotationkI,
          JsonConstants.drivetrainConstants.driveRotationkD);
  
  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("");
  private DoubleSubscriber reefLocationSelector = table.getDoubleTopic("reefTarget").subscribe(-1);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.alignmentSupplier = null;
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
            new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
        PP_CONFIG,
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
        this);
    Pathfinding.setPathfinder(new LocalADStarAK());
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
    PathfindingCommand.warmupCommand().schedule();

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
  }

  @Override
  public void periodic() {
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

    // Log empty setpoint states when disabled
    if (DriverStation.isDisabled()) {
      Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
      Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
    }

    // OTF Command
    if (driveToPose != null) {
      Logger.recordOutput("Drive/OTF/OnTheFlyCommandStatus", this.driveToPose.isScheduled());

      // cancel path following command once OTF cancelled (likely via trigger)
      if (!isOTF) {
        driveToPose.cancel();
      }
    }

    if (isDriveCloseToFinalLineupPose() && isOTF) {
      this.setOTF(false);
      driveToPose.cancel();
      this.setLiningUp(true);
    }

    if (isLiningUp) {
      this.LineupWithReefLocation();
    }
    
    // check for update from reef touchscreen
    this.updateDesiredLocationFromNetworkTables();

    // run velocity if not disabled
    if (!DriverStation.isTest() && !DriverStation.isDisabled()) {
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
    gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);
  }

  public void testPeriodic() {
    switch (TestModeManager.getTestMode()) {
      case DriveLineupTuning:
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              this.setAlongTrackPID(pid[0], pid[1], pid[2]);
            },
            alongTrackkP,
            alongTrackkI,
            alongTrackkD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              this.setCrossTrackPID(pid[0], pid[1], pid[2]);
            },
            crossTrackkP,
            crossTrackkI,
            crossTrackkD);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              this.setRotationLineupPID(pid[0], pid[1], pid[2]);
            },
            rotationkP,
            rotationkI,
            rotationkD);
        break;
      default:
        break;
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
    if (fieldCentric) {
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

  /**
   * sets lineup along track pid gains
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   */
  public void setAlongTrackPID(double kP, double kI, double kD) {
    if (kP < 0) {
      kP = this.driveAlongTrackLineupController.getP();
    }
    if (kI < 0) {
      kI = this.driveAlongTrackLineupController.getI();
    }
    if (kD < 0) {
      kD = this.driveAlongTrackLineupController.getD();
    }
    this.driveAlongTrackLineupController = new PIDController(kP, kI, kD);
  }

  /**
   * sets lineup cross track pid gains
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   */
  public void setCrossTrackPID(double kP, double kI, double kD) {
    if (kP < 0) {
      kP = this.driveCrossTrackLineupController.getP();
    }
    if (kI < 0) {
      kI = this.driveCrossTrackLineupController.getI();
    }
    if (kD < 0) {
      kD = this.driveCrossTrackLineupController.getD();
    }
    this.driveCrossTrackLineupController = new PIDController(kP, kI, kD);
  }

  /**
   * sets lineup rotation pid gains
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   */
  public void setRotationLineupPID(double kP, double kI, double kD) {
    if (kP < 0) {
      kP = this.rotationController.getP();
    }
    if (kI < 0) {
      kI = this.rotationController.getI();
    }
    if (kD < 0) {
      kD = this.rotationController.getD();
    }
    this.rotationController = new PIDController(kP, kI, kD);
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
   * sets isOTF of robot true will cause robot to create a path from current location to the set
   * PathLocation
   *
   * @param isOTF boolean telling robot if it should create a OTF path
   */
  public void setOTF(boolean isOTF) {
    this.isOTF = isOTF;

    if (isOTF) {
      this.driveToPose = this.getDriveToPoseCommand();
      if (this.driveToPose != null) {
        this.driveToPose.schedule();
      }
    }
  }

  /**
   * checks if drive is currently following an on the fly path
   *
   * @return state of OTF following
   */
  @AutoLogOutput(key = "Drive/OTF/isOTF")
  public boolean isDriveOTF() {
    return isOTF;
  }

  /**
   * checks if robot pose is sufficiently close to desired pose
   *
   * @return true if robot pose is close to desired pose
   */
  public boolean isDriveCloseToFinalLineupPose() {
    // relative to transforms first pose into distance from desired pose
    // then get distance between poses (if less than 0.1 meters we are good)
    return this.getPose()
            .relativeTo(this.findOTFPoseFromDesiredLocation())
            .getTranslation()
            .getNorm()
        < JsonConstants.drivetrainConstants.otfPoseDistanceLimit;
  }

  /**
   * sets isLiningUp of robot true will cause robot to gather distance from reef tag and drive
   * towards it PathLocation
   *
   * @param isLiningUp boolean telling robot if it should create a OTF path
   */
  public void setLiningUp(boolean isLiningUp) {
    this.isLiningUp = isLiningUp;
  }

  /**
   * checks if drive is currently lining up to a reef
   *
   * @return state of lining up
   */
  @AutoLogOutput(key = "Drive/Lineup/isLiningUp")
  public boolean isDriveLiningUp() {
    return isLiningUp;
  }

  /**
   * checks if desired locaiton is set to a reef location
   *
   * @return true if location is reef; false otherwise (processor / coral station)
   */
  public boolean isDesiredLocationReef() {
    if (desiredLocation == DesiredLocation.CoralStationLeft
        || desiredLocation == DesiredLocation.CoralStationRight
        || desiredLocation == DesiredLocation.Processor) {
      return false;
    }
    return true;
  }

  /**
   * allows drive to be controlled by on the fly / landing zone alignment
   *
   * @param autoAlignment true allows drive to go into otf and alignment
   */
  public void setAutoAlignment(boolean autoAlignment) {
    if (autoAlignment) {
      if (isDriveCloseToFinalLineupPose() && isDesiredLocationReef()) {
        this.setLiningUp(true);
      } else {
        this.setOTF(true);
      }
    } else {
      setOTF(false);
      setLiningUp(false);
    }
  }

  /**
   * sets desired path location calling this and then setting OTF to true will cause robot to drive
   * path from current pose to the location
   *
   * @param location desired location for robot to pathfind to
   */
  public void setDesiredLocation(DesiredLocation location) {
    this.desiredLocation = location;
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
  public void updateDesiredLocationFromNetworkTables() {
    double desiredIndex = reefLocationSelector.get();
    if (desiredIndex == -1) {
      return;
    }
    if (locationArray[(int) desiredIndex] != desiredLocation) {
      if (isOTF) {
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

  /**
   * updates desired path location (for when OTF is already running) this will cancel old command
   * and generate a new OTF path to run
   *
   * @param location desired location for robot to pathfind to
   */
  public void updateDesiredLocation(DesiredLocation location) {
    this.setDesiredLocation(location);

    if (isOTF) {
      this.driveToPose.cancel();
      this.driveToPose = this.getDriveToPoseCommand();
      this.driveToPose.schedule();
    }
  }

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public Pose2d findOTFPoseFromDesiredLocation() {
    switch (this.desiredLocation) {
        // NOTE: pairs of reef sides (ie 0 and 1) will have the same otf pose (approximately 0.5-1
        // meter away from center of tag)
      case Reef0:
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? JsonConstants.redFieldLocations.reef0
            : new Pose2d();
      case Reef1:
        return new Pose2d(Meters.of(14.350), Meters.of(4.0), new Rotation2d(Degrees.of(180)));
      case CoralStationRight:
        return new Pose2d(16.0, 6.6, new Rotation2d(0.0));
        // return new Pose2d(1.2, 1, Rotation2d.fromRadians(1));
      case CoralStationLeft:
        return new Pose2d(1.2, 7.0, Rotation2d.fromRadians(-1));
      default:
        // no location set, so don't allow drive to run OTF
        this.setOTF(false);
        return null;
    }
  }

  /**
   * gets the path from current pose to the desired pose found from location
   *
   * @return command that drive can schedule to follow the path found
   */
  public Command getDriveToPoseCommand() {
    Pose2d targetPose = findOTFPoseFromDesiredLocation();

    if (targetPose == null) {
      return null;
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
            JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
            JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
            JsonConstants.drivetrainConstants.OTFMaxAngularAccel);

    return AutoBuilder.pathfindToPose(targetPose, constraints, 0.0);
  }

  /**
   * checks if driver station alliance is red
   *
   * @return true if alliance is red
   */
  public boolean isAllianceRed() {
    return DriverStation.getAlliance().orElse(Alliance.Blue).equals(Alliance.Red);
  }

  /**
   * gets tag to use for final alignment with vision
   *
   * @return int representing tag id to use
   */
  public int getTagIdForReef() {
    boolean allianceRed = this.isAllianceRed();
    switch (desiredLocation) {
      case Reef0:
      case Reef1:
        return allianceRed ? 10 : 21;
      case Reef2:
      case Reef3:
        return allianceRed ? 9 : 22;
      case Reef4:
      case Reef5:
        return allianceRed ? 8 : 17;
      case Reef6:
      case Reef7:
        return allianceRed ? 7 : 18;
      case Reef8:
      case Reef9:
        return allianceRed ? 6 : 19;
      case Reef10:
      case Reef11:
        return allianceRed ? 11 : 20;
      default:
        return -1;
    }
  }

  /**
   * gets camera index for vision single tag lineup
   *
   * @return 0 for Front Left camera; 1 for Front Right camera
   */
  public int getCameraIndexForLineup() {
    switch (desiredLocation) {
        // Right Side of reef side (align to left camera)
      case Reef0:
      case Reef2:
      case Reef4:
      case Reef6:
      case Reef8:
      case Reef10:
        return FrontLeftCameraIndex;
        // Left side of reef side (align to right camera)
      case Reef1:
      case Reef3:
      case Reef5:
      case Reef7:
      case Reef9:
      case Reef11:
        return FrontRightCameraIndex;
      default:
        return -1;
    }
  }

  /**
   * gets rotation for each side of hexagonal reef for lineup
   *
   * @return Rotation2d representing desired rotation for lineup
   */
  public Rotation2d getRotationForReefSide() {
    switch (desiredLocation) {
      case Reef0:
        return new Rotation2d();
      case Reef1:
        return new Rotation2d();
      case Reef2:
        return new Rotation2d();
      case Reef3:
        return new Rotation2d();
      case Reef4:
        return new Rotation2d();
      case Reef5:
        return new Rotation2d();
      case Reef6:
        return new Rotation2d();
      case Reef7:
        return new Rotation2d();
      case Reef8:
        return new Rotation2d();
      case Reef9:
        return new Rotation2d();
      case Reef10:
        return new Rotation2d();
      case Reef11:
        return new Rotation2d();
      default:
        return new Rotation2d();
    }
  }

  /** take over goal speeds to align to reef exactly */
  public void LineupWithReefLocation() {
    int tagId = this.getTagIdForReef();
    int cameraIndex = this.getCameraIndexForLineup();

    if (tagId == -1 || cameraIndex == -1 || alignmentSupplier == null) {
      // cancel lineup or whatever
      this.setLiningUp(false);
      return;
    }

    DistanceToTag observation = alignmentSupplier.get(tagId, cameraIndex, 0, 0);

    if (!observation.isValid()) {
      return;
    }

    // give to PID Controllers and setGoalSpeeds (robotCentric)
    double vx = driveAlongTrackLineupController.calculate(observation.alongTrackDistance());
    double vy = driveCrossTrackLineupController.calculate(observation.crossTrackDistance());
    double omega =
        rotationController.calculate(
            this.getRotation().getRadians(), this.getRotationForReefSide().getRadians());

    Logger.recordOutput("Drive/Lineup/AlongTrackDistance", observation.alongTrackDistance());
    Logger.recordOutput("Drive/Lineup/CrossTrackDistance", observation.crossTrackDistance());

    this.setGoalSpeeds(new ChassisSpeeds(vx, vy, omega), false);
  }

  /** Runs the drive at the desired speeds set in (@Link setGoalSpeeds) */
  public void runVelocity() {
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
  private ChassisSpeeds getChassisSpeeds() {
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
