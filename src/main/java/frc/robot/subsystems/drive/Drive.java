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
import coppercore.wpilib_interface.DriveTemplate;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
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

  private boolean isOTF = false;

  private Command driveToPose = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();
  private NetworkTable table = inst.getTable("");
  private DoubleSubscriber reefLocationSelector = table.getDoubleTopic("reefTarget").subscribe(-1);

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
    Logger.recordOutput("Drive/OnTheFly", isOTF);
    if (driveToPose != null) {
      Logger.recordOutput("Drive/OnTheFlyCommandStatus", this.driveToPose.isScheduled());

      // cancel path following command once OTF cancelled (likely via trigger)
      if (!isOTF) {
        driveToPose.cancel();
      }
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
      Logger.recordOutput("Drive/DesiredOTFSpeeds", speeds);
      this.goalSpeeds = speeds;
    }
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
  public boolean isDriveOTF() {
    return isOTF;
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

  public void updateDesiredLocationFromNetworkTables() {
    double desiredIndex = reefLocationSelector.get();
    if (desiredIndex == -1) {
      this.setOTF(false);
      return;
    }
    desiredIndex = desiredIndex - 1;
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
   * updates desired path location (for when OTF is already running) this will cancel old command
   * and generate a new OTF path to run
   *
   * @param locationIndex desired location index for robot to pathfind to (sent from
   *     DesiredLocationSelector)
   */
  public void updateDesiredLocation(int locationIndex) {
    this.setDesiredLocation(locationIndex);

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
  public Pose2d findOTFPoseFromPathLocation() {
    switch (this.desiredLocation) {
        // reef 0 and 1 will have the same path
        // NOTE: use PathPlannerPath.getStartingHolonomicPose to find pose for reef lineup if wanted
      case Reef0:
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
            ? JsonConstants.redFieldLocations.reef0
            : new Pose2d();
      case Reef1:
        return null;
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
    Pose2d targetPose = findOTFPoseFromPathLocation();

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
}
