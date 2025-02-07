package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.vision.VisionLocalizer.DistanceToTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.drive.Drive.VisionAlignment;
import org.littletonrobotics.junction.Logger;

public class LineupState implements PeriodicStateInterface {
  private Drive drive;

  private DistanceToTag latestObservation;
  private int observationAge;

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
          "DriveLineupGains/rotationkD", JsonConstants.drivetrainConstants.driveRotationkD);

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

  private Constraints driveAlongTrackProfileConstraints =
      new Constraints(
          JsonConstants.drivetrainConstants.driveAlongTrackVelocity,
          JsonConstants.drivetrainConstants.driveAlongTrackVelocity);
  private TrapezoidProfile driveAlongTrackProfile =
      new TrapezoidProfile(driveAlongTrackProfileConstraints);

  private PIDController rotationController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveRotationkP,
          JsonConstants.drivetrainConstants.driveRotationkI,
          JsonConstants.drivetrainConstants.driveRotationkD);

  public LineupState(Drive drive) {
    this.drive = drive;
    rotationController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
  }

  public void onEntry(Transition transition) {
    // cancel rotation lock on center
    drive.disableAlign();
  }

  /**
   * gets rotation for each side of hexagonal reef for lineup
   *
   * @return Rotation2d representing desired rotation for lineup
   */
  public Rotation2d getRotationForReefSide() {
    switch (drive.getDesiredLocation()) {
      case Reef0:
      case Reef1:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReef01Rotation
            : JsonConstants.blueFieldLocations.blueReef01Rotation;
      case Reef2:
      case Reef3:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReef23Rotation
            : JsonConstants.blueFieldLocations.blueReef23Rotation;
      case Reef4:
      case Reef5:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReef45Rotation
            : JsonConstants.blueFieldLocations.blueReef45Rotation;
      case Reef6:
      case Reef7:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReef67Rotation
            : JsonConstants.blueFieldLocations.blueReef67Rotation;
      case Reef8:
      case Reef9:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReef89Rotation
            : JsonConstants.blueFieldLocations.blueReef89Rotation;
      case Reef10:
      case Reef11:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReef1011Rotation
            : JsonConstants.blueFieldLocations.blueReef1011Rotation;
      default:
        return new Rotation2d();
    }
  }

  /**
   * checks if lineup is within 0.01 of tag + offsets
   *
   * @return true if error is small enoguh
   */
  public boolean lineupFinished() {
    return latestObservation != null
        && (latestObservation.alongTrackDistance() < 0.01
            && latestObservation.crossTrackDistance() < 0.01);
  }

  public void periodic() {
    // run test through here if in test mode
    if (DriverStation.isTest()) {
      this.testPeriodic();
    }
    this.LineupWithReefLocation();

    if (lineupFinished()) {
      drive.fireTrigger(DriveTrigger.FinishLineup);
    }
  }

  /**
   * gets tag to use for final alignment with vision
   *
   * @return int representing tag id to use
   */
  public int getTagIdForReef() {
    boolean allianceRed = drive.isAllianceRed();
    switch (drive.getDesiredLocation()) {
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
   * gets cross track offset for lineup
   *
   * @param cameraIndex camera to check offset
   * @return offset for camera
   */
  public Double getCrossTrackOffset(int cameraIndex) {
    if (cameraIndex == 0) {
      return JsonConstants.drivetrainConstants.driveCrossTrackFrontRightOffset;
    }
    return 0.0; // front left offset (not added)
  }

  /**
   * gets camera index for vision single tag lineup
   *
   * @return 0 for Front Left camera; 1 for Front Right camera
   */
  public int getCameraIndexForLineup() {
    switch (drive.getDesiredLocation()) {
        // Right Side of reef side (align to left camera)
      case Reef0:
      case Reef2:
      case Reef4:
      case Reef6:
      case Reef8:
      case Reef10:
        return JsonConstants.visionConstants.FrontLeftCameraIndex;
        // Left side of reef side (align to right camera)
      case Reef1:
      case Reef3:
      case Reef5:
      case Reef7:
      case Reef9:
      case Reef11:
        return JsonConstants.visionConstants.FrontRightCameraIndex;
      default:
        return -1;
    }
  }

  /** take over goal speeds to align to reef exactly */
  public void LineupWithReefLocation() {
    int tagId = this.getTagIdForReef();
    int cameraIndex = this.getCameraIndexForLineup();

    if (tagId == -1 || cameraIndex == -1) {
      // TODO: check if this might be false first time, but on another loop true
      // drive.fireTrigger(DriveTrigger.CancelLineup);
      return;
    }

    VisionAlignment alignmentSupplier = drive.getVisionAlignment();

    if (alignmentSupplier == null) {
      return;
    }

    DistanceToTag observation =
        alignmentSupplier.get(
            tagId,
            cameraIndex,
            this.getCrossTrackOffset(cameraIndex),
            JsonConstants.drivetrainConstants.driveAlongTrackOffset);

    if (!observation.isValid()) {
      if (latestObservation != null && observationAge < 5) {
        observation = latestObservation;
        observationAge++;
      } else {
        return;
      }
    } else {
      latestObservation = observation;
      observationAge = 0;
    }

    Logger.recordOutput("Drive/Lineup/AlongTrackDistance", observation.alongTrackDistance());
    Logger.recordOutput("Drive/Lineup/CrossTrackDistance", observation.crossTrackDistance());
    Logger.recordOutput("Drive/Lineup/IsObservationValid", observation.isValid());

    // give to PID Controllers and setGoalSpeeds (robotCentric)
    double vx =
        JsonConstants.drivetrainConstants.driveAlongTrackMultiplier
            * driveAlongTrackLineupController.calculate(observation.alongTrackDistance());
    // ChassisSpeeds speeds = getChassisSpeeds();
    // Pose2d pose = getPose();
    // double velocityToTarget = (pose.getX() * speeds.vxMetersPerSecond + pose.getY() *
    // speeds.vyMetersPerSecond) / Math.sqrt(pose.getX() * pose.getX() + pose.getY() * pose.getY());
    // double velocityToTarget = -getChassisSpeeds().vxMetersPerSecond;
    // Logger.recordOutput("Drive/Lineup/velocityToTarget", velocityToTarget);
    // double vx =
    //     -driveAlongTrackProfile.calculate(
    //             0.02,
    //             new State(observation.alongTrackDistance(), velocityToTarget),
    //             new State(0, 0))
    //         .velocity;
    double vy = driveCrossTrackLineupController.calculate(observation.crossTrackDistance());
    double omega =
        rotationController.calculate(
            drive.getRotation().getRadians(), this.getRotationForReefSide().getRadians());

    drive.setGoalSpeeds(new ChassisSpeeds(vx, vy, omega), false);
  }

  /**
   * sets lineup along track pid gains
   *
   * @param kP proportional gain
   * @param kI integral gain
   * @param kD derivative gain
   */
  public void setAlongTrackPID(double kP, double kI, double kD) {
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
    this.rotationController = new PIDController(kP, kI, kD);
    this.rotationController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
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
}
