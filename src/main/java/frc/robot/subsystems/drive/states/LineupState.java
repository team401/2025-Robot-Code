package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.vision.VisionLocalizer.DistanceToTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ModeConstants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.drive.Drive.VisionAlignment;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class LineupState implements PeriodicStateInterface {
  private Drive drive;

  private DistanceToTag latestObservation;
  private int observationAge;

  // along track pid test mode
  private LoggedTunableNumber alongTrackKp =
      new LoggedTunableNumber(
          "DriveLineupGains/AlongTrackKp", JsonConstants.drivetrainConstants.driveAlongTrackKp);
  private LoggedTunableNumber alongTrackKi =
      new LoggedTunableNumber(
          "DriveLineupGains/AlongTrackKi", JsonConstants.drivetrainConstants.driveAlongTrackKi);
  private LoggedTunableNumber alongTrackKd =
      new LoggedTunableNumber(
          "DriveLineupGains/AlongTrackKd", JsonConstants.drivetrainConstants.driveAlongTrackKd);

  // cross tack pid test mode
  private LoggedTunableNumber crossTrackKp =
      new LoggedTunableNumber(
          "DriveLineupGains/CrossTrackKp", JsonConstants.drivetrainConstants.driveCrossTrackKp);
  private LoggedTunableNumber crossTrackKi =
      new LoggedTunableNumber(
          "DriveLineupGains/CrossTrackKi", JsonConstants.drivetrainConstants.driveCrossTrackKi);
  private LoggedTunableNumber crossTrackKd =
      new LoggedTunableNumber(
          "DriveLineupGains/CrossTrackKd", JsonConstants.drivetrainConstants.driveCrossTrackKd);

  // rotation pid test mode
  private LoggedTunableNumber rotationkP =
      new LoggedTunableNumber(
          "DriveLineupGains/rotationkP", JsonConstants.drivetrainConstants.driveRotationKp);
  private LoggedTunableNumber rotationkI =
      new LoggedTunableNumber(
          "DriveLineupGains/rotationkI", JsonConstants.drivetrainConstants.driveRotationKi);
  private LoggedTunableNumber rotationkD =
      new LoggedTunableNumber(
          "DriveLineupGains/rotationkD", JsonConstants.drivetrainConstants.driveRotationKd);

  private PIDController driveAlongTrackLineupController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveAlongTrackKp,
          JsonConstants.drivetrainConstants.driveAlongTrackKi,
          JsonConstants.drivetrainConstants.driveAlongTrackKd);
  private PIDController driveCrossTrackLineupController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveCrossTrackKp,
          JsonConstants.drivetrainConstants.driveCrossTrackKi,
          JsonConstants.drivetrainConstants.driveCrossTrackKd);

  private Constraints driveAlongTrackProfileConstraints =
      new Constraints(
          JsonConstants.drivetrainConstants.driveAlongTrackVelocity,
          JsonConstants.drivetrainConstants.driveAlongTrackVelocity);

  private PIDController rotationController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveRotationKp,
          JsonConstants.drivetrainConstants.driveRotationKi,
          JsonConstants.drivetrainConstants.driveRotationKd);

  LinearFilter alongTrackFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  LinearFilter crossTrackFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

  private boolean otherCameraTried = false;

  public LineupState(Drive drive) {
    this.drive = drive;
    rotationController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);
  }

  public void onEntry(Transition transition) {
    // cancel rotation lock on center
    drive.disableAlign();

    // dont rely on previous estimates
    latestObservation = new DistanceToTag(0.0, 0.0, true);
    observationAge = 0;

    // begin warming up elevator/wrist when lineup starts
    if (ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
    }
  }

  public void onExit(Transition transition) {
    drive.setDriveLinedUp(false);
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
        && (latestObservation.alongTrackDistance() < 0.05
            && Math.abs(latestObservation.crossTrackDistance()) < 0.01);
  }

  public void periodic() {
    // run test through here if in test mode
    if (DriverStation.isTest()) {
      this.testPeriodic();
    }
    this.LineupWithReefLocation();

    drive.setDriveLinedUp(lineupFinished());
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
    if (cameraIndex == JsonConstants.visionConstants.FrontRightCameraIndex) {
      return JsonConstants.drivetrainConstants.driveCrossTrackFrontRightOffset;
    } else {
      return JsonConstants.drivetrainConstants.driveCrossTrackFrontLeftOffset;
    }
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

  public double getAlongTrackVelocity(double alongTrackDistance) {
    if (alongTrackDistance < 0
        && (ModeConstants.currentMode == Mode.MAPLESIM || ModeConstants.currentMode == Mode.SIM)) {
      return 0;
    }

    if (Math.abs(alongTrackDistance) < 0.01) {
      return 0;
    }

    // if (alongTrackDistance < 0.2) {
    //   return -1
    //       * JsonConstants.drivetrainConstants.driveAlongTrackMultiplier
    //       * driveAlongTrackLineupController.calculate(alongTrackDistance);
    // }

    double multiplier = 1.0;

    if (Math.abs(alongTrackDistance) < 0.1) {
      multiplier = 0.5;
    }

    double sign = Math.signum(alongTrackDistance);

    double rawVelocity =
        Math.sqrt(
            2
                * JsonConstants.drivetrainConstants.lineupMaxAcceleration
                * Math.abs(alongTrackDistance));
    return sign
        * multiplier
        * Math.min(rawVelocity, JsonConstants.drivetrainConstants.lineupMaxVelocity);
  }

  public double getAlongTrackVelocityReductionFactor(double crossTrackDistance) {
    return 2 * (0.5 - Math.abs(crossTrackDistance));
  }

  /**
   * check the other camera index for a valid observation
   *
   * @param alignmentSupplier vision supplier
   * @param tagId tag to check for
   * @param cameraIndex the camera index already tried
   * @return
   */
  public DistanceToTag tryOtherCamera(
      VisionAlignment alignmentSupplier, int tagId, int cameraIndex) {
    int otherCameraIndex = cameraIndex == 0 ? 1 : 0;
    otherCameraTried = true;
    DistanceToTag observationOtherCamera =
        alignmentSupplier.get(
            tagId,
            otherCameraIndex,
            this.getCrossTrackOffset(otherCameraIndex),
            JsonConstants.drivetrainConstants.driveAlongTrackOffset);
    if (observationOtherCamera.isValid()) {
      return observationOtherCamera;
    }

    return null;
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
      }
      // } else if (!otherCameraTried) {
      //   // check other camera (for when otf comes in from other side)
      //   DistanceToTag otherCameraObs = tryOtherCamera(alignmentSupplier, tagId, cameraIndex);

      //   if (otherCameraObs != null) {
      //     observation = otherCameraObs;
      //   }
      // }
      else {
        drive.fireTrigger(DriveTrigger.BeginOTF);
      }
    } else {
      latestObservation = observation;
      observationAge = 0;
    }

    double alongTrackDistanceFiltered =
        alongTrackFilter.calculate(observation.alongTrackDistance());
    double crossTrackDistanceFiltered =
        crossTrackFilter.calculate(observation.crossTrackDistance());

    Logger.recordOutput("Drive/Lineup/AlongTrackDistance", observation.alongTrackDistance());
    Logger.recordOutput("Drive/Lineup/CrossTrackDistance", observation.crossTrackDistance());
    Logger.recordOutput("Drive/Lineup/AlongTrackDistanceFiltered", alongTrackDistanceFiltered);
    Logger.recordOutput("Drive/Lineup/CrossTrackDistanceFiltered", crossTrackDistanceFiltered);
    Logger.recordOutput("Drive/Lineup/IsObservationValid", observation.isValid());

    // give to PID Controllers and setGoalSpeeds (robotCentric)
    if (!lineupFinished()) {
      double vx =
          JsonConstants.drivetrainConstants.driveAlongTrackMultiplier
              * getAlongTrackVelocityReductionFactor(observation.crossTrackDistance())
              * getAlongTrackVelocity(alongTrackDistanceFiltered);
      double vy = driveCrossTrackLineupController.calculate(crossTrackDistanceFiltered);
      double omega =
          rotationController.calculate(
              drive.getRotation().getRadians(), this.getRotationForReefSide().getRadians());

      drive.setGoalSpeeds(new ChassisSpeeds(vx, vy, omega), false);
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
            alongTrackKp,
            alongTrackKi,
            alongTrackKd);
        LoggedTunableNumber.ifChanged(
            hashCode(),
            (pid) -> {
              this.setCrossTrackPID(pid[0], pid[1], pid[2]);
            },
            crossTrackKp,
            crossTrackKi,
            crossTrackKd);
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
