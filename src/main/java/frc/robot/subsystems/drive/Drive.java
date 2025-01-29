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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
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
                  JsonConstants.drivetrainConstants.FrontLeft.LocationX,
                  JsonConstants.drivetrainConstants.FrontLeft.LocationY),
              Math.hypot(
                  JsonConstants.drivetrainConstants.FrontRight.LocationX,
                  JsonConstants.drivetrainConstants.FrontRight.LocationY)),
          Math.max(
              Math.hypot(
                  JsonConstants.drivetrainConstants.BackLeft.LocationX,
                  JsonConstants.drivetrainConstants.BackLeft.LocationY),
              Math.hypot(
                  JsonConstants.drivetrainConstants.BackRight.LocationX,
                  JsonConstants.drivetrainConstants.BackRight.LocationY)));

  // PathPlanner config constants
  private static final double ROBOT_MASS_KG = 74.088;
  private static final double ROBOT_MOI = 6.883;
  private static final double WHEEL_COF = 1.2;
  private static final RobotConfig PP_CONFIG =
      new RobotConfig(
          ROBOT_MASS_KG,
          ROBOT_MOI,
          new ModuleConfig(
              JsonConstants.drivetrainConstants.FrontLeft.WheelRadius,
              JsonConstants.drivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond),
              WHEEL_COF,
              DCMotor.getKrakenX60Foc(1)
                  .withReduction(JsonConstants.drivetrainConstants.FrontLeft.DriveMotorGearRatio),
              JsonConstants.drivetrainConstants.FrontLeft.SlipCurrent,
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

  private boolean isOTF = false;

  private boolean isLiningUp = false;

  private Command driveToPose = null;

  private VisionAlignment alignmentSupplier;

  private PIDController driveLineupController = new PIDController(20, 0,0);
  private PIDController rotationController = new PIDController(20, 0, 0);

  public Drive(
      GyroIO gyroIO,
      ModuleIO flModuleIO,
      ModuleIO frModuleIO,
      ModuleIO blModuleIO,
      ModuleIO brModuleIO) {
    this.alignmentSupplier = null;
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0, JsonConstants.drivetrainConstants.FrontLeft);
    modules[1] = new Module(frModuleIO, 1, JsonConstants.drivetrainConstants.FrontRight);
    modules[2] = new Module(blModuleIO, 2, JsonConstants.drivetrainConstants.BackLeft);
    modules[3] = new Module(brModuleIO, 3, JsonConstants.drivetrainConstants.BackRight);

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
      Logger.recordOutput("Drive/OnTheFlyCommandStatus", this.driveToPose.isScheduled());

      // cancel path following command once OTF cancelled (likely via trigger)
      if (!isOTF) {
        driveToPose.cancel();
      }
    }

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
      this.driveToPose.schedule();
    }
  }

  /**
   * checks if drive is currently following an on the fly path
   *
   * @return state of OTF following
   */
  @AutoLogOutput(key = "Drive/isOTF")
  public boolean isDriveOTF() {
    return isOTF;
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
  @AutoLogOutput(key = "Drive/isLiningUp")
  public boolean isDriveLiningUp() {
    return isLiningUp;
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

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public Pose2d findOTFPoseFromPathLocation() {
    switch (this.desiredLocation) {
        // reef 0 and 1 will have the same path
        // NOTE: use PathPlannerPath.getStartingHolonomicPose to find pose for reef lineup if wanted
      case Reef0:
        return new Pose2d();
      case Reef1:
        return new Pose2d();
      case CoralStationRight:
        return new Pose2d(1.2, 1, Rotation2d.fromRadians(1));
      case CoralStationLeft:
        return new Pose2d(1.2, 7.0, Rotation2d.fromRadians(-1));
      default:
        // no location set, so don't allow drive to run OTF
        this.setOTF(false);
        return new Pose2d();
    }
  }

  /**
   * gets the path from current pose to the desired pose found from location
   *
   * @return command that drive can schedule to follow the path found
   */
  public Command getDriveToPoseCommand() {
    Pose2d targetPose = findOTFPoseFromPathLocation();

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(3.0, 4.0, Units.degreesToRadians(540), Units.degreesToRadians(720));

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
        return allianceRed ? 10 : 21;
      case Reef1:
        return allianceRed ? 10 : 21;
      case Reef2:
        return allianceRed ? 9 : 22;
      case Reef3:
        return allianceRed ? 9 : 22;
      case Reef4:
        return allianceRed ? 8 : 17;
      case Reef5:
        return allianceRed ? 8 : 17;
      case Reef6:
        return allianceRed ? 7 : 18;
      case Reef7:
        return allianceRed ? 7 : 18;
      case Reef8:
        return allianceRed ? 6 : 19;
      case Reef9:
        return allianceRed ? 6 : 19;
      case Reef10:
        return allianceRed ? 11 : 20;
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
      case Reef0:
        return 0;
      case Reef1:
        return 1;
      case Reef2:
        return 0;
      case Reef3:
        return 1;
      case Reef4:
        return 0;
      case Reef5:
        return 1;
      case Reef6:
        return 0;
      case Reef7:
        return 1;
      case Reef8:
        return 0;
      case Reef9:
        return 1;
      case Reef10:
        return 0;
      case Reef11:
        return 1;
      default:
        return -1;
    }
  }

  /**
   * gets rotation for each side of hexagonal reef for lineup
   * 
   * @return Rotation2d representing desired rotation for lineup
   */
  public Rotation2d getRotationForReefSide () {
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
    double vx = driveLineupController.calculate(observation.alongTrackDistance());
    double vy = driveLineupController.calculate(observation.crossTrackDistance());
    double omega = rotationController.calculate(this.getRotation().getRadians(), this.getRotationForReefSide().getRadians());

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
          JsonConstants.drivetrainConstants.FrontLeft.LocationX,
          JsonConstants.drivetrainConstants.FrontLeft.LocationY),
      new Translation2d(
          JsonConstants.drivetrainConstants.FrontRight.LocationX,
          JsonConstants.drivetrainConstants.FrontRight.LocationY),
      new Translation2d(
          JsonConstants.drivetrainConstants.BackLeft.LocationX,
          JsonConstants.drivetrainConstants.BackLeft.LocationY),
      new Translation2d(
          JsonConstants.drivetrainConstants.BackRight.LocationX,
          JsonConstants.drivetrainConstants.BackRight.LocationY)
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
