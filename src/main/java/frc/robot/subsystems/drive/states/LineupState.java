package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import coppercore.parameter_tools.LoggedTunableNumber;
import coppercore.vision.VisionLocalizer.DistanceToTag;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.StrategyManager;
import frc.robot.StrategyManager.AutonomyMode;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ModeConstants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.drive.Drive.VisionAlignment;
import frc.robot.subsystems.drive.ReefLineupUtil;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class LineupState implements PeriodicStateInterface {
  private Drive drive;

  private DistanceToTag latestObservation;
  private boolean hadObservationYet = false;
  private int observationAge;
  private DesiredLocation lastReefLocation = DesiredLocation.Reef0;
  private boolean usingOtherCamera = false;

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

  private PIDController driveCrossTrackLineupController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveCrossTrackKp,
          JsonConstants.drivetrainConstants.driveCrossTrackKi,
          JsonConstants.drivetrainConstants.driveCrossTrackKd);

  private PIDController driveCrossTrackOtherCameraLineupController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveCrossTrackOtherCameraKp,
          JsonConstants.drivetrainConstants.driveCrossTrackOtherCameraKi,
          JsonConstants.drivetrainConstants.driveCrossTrackOtherCameraKd);

  private PIDController rotationController =
      new PIDController(
          JsonConstants.drivetrainConstants.driveRotationKp,
          JsonConstants.drivetrainConstants.driveRotationKi,
          JsonConstants.drivetrainConstants.driveRotationKd);

  LinearFilter alongTrackFilter = LinearFilter.singlePoleIIR(0.2, 0.02);
  LinearFilter crossTrackFilter = LinearFilter.singlePoleIIR(0.2, 0.02);

  Pose2d poseAtLastObservation = null; // Initialize to null so we don't somehow lineup off of 0, 0

  public LineupState(Drive drive) {
    this.drive = drive;
    rotationController.enableContinuousInput(-Math.PI / 2, Math.PI / 2);

    // Log adjustment on init so that it can be added to scope before lineup starts
    Logger.recordOutput("Drive/Lineup/Adjustment", new Translation2d());
    Logger.recordOutput("Drive/Lineup/FinalPose", new Pose2d());
  }

  public void onEntry(Transition transition) {
    // Drive isn't done lining up
    drive.setDriveLinedUp(false);

    // cancel rotation lock on center
    drive.disableAlign();

    // dont rely on previous estimates
    latestObservation = null;
    observationAge = 0;

    hadObservationYet = false;

    lastReefLocation = drive.getDesiredLocation();

    poseAtLastObservation = drive.getPose();

    // begin warming up elevator/wrist when lineup starts
    if (ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
      System.out.println("StartWarmup!");
    }
  }

  public void onExit(Transition transition) {
    drive.setDriveLinedUp(true);
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
      case Algae0:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF0Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF0Rotation;
      case Reef2:
      case Reef3:
      case Algae1:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF2Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF2Rotation;
      case Reef4:
      case Reef5:
      case Algae2:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF4Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF4Rotation;
      case Reef6:
      case Reef7:
      case Algae3:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF6Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF6Rotation;
      case Reef8:
      case Reef9:
      case Algae4:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF8Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF8Rotation;
      case Reef10:
      case Reef11:
      case Algae5:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF10Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF10Rotation;
      default:
        return new Rotation2d();
    }
  }

  public boolean checkForSideSwitch() {
    switch (drive.getDesiredLocation()) {
      case Reef0:
        return lastReefLocation == DesiredLocation.Reef1;
      case Reef1:
        return lastReefLocation == DesiredLocation.Reef0;
      case Reef2:
        return lastReefLocation == DesiredLocation.Reef3;
      case Reef3:
        return lastReefLocation == DesiredLocation.Reef2;
      case Reef4:
        return lastReefLocation == DesiredLocation.Reef5;
      case Reef5:
        return lastReefLocation == DesiredLocation.Reef4;
      case Reef6:
        return lastReefLocation == DesiredLocation.Reef7;
      case Reef7:
        return lastReefLocation == DesiredLocation.Reef6;
      case Reef8:
        return lastReefLocation == DesiredLocation.Reef9;
      case Reef9:
        return lastReefLocation == DesiredLocation.Reef8;
      case Reef10:
        return lastReefLocation == DesiredLocation.Reef11;
      case Reef11:
        return lastReefLocation == DesiredLocation.Reef10;
      default:
        return false;
    }
  }

  /** checks for reef location update and handles state accordingly */
  public void checkForReefUpdate() {
    // if we changed sides throw a invalid observation out so lineup doesnt think its finished and
    // we can move on to other side
    if (checkForSideSwitch()) {
      lastReefLocation = drive.getDesiredLocation();
      latestObservation = new DistanceToTag(0, 0, false);
    }
    // if we changed locations and its not to the other side we need to go back to OTF to get
    // within tag distance
    else if (drive.getDesiredLocation() != lastReefLocation && !checkForSideSwitch()) {
      drive.fireTrigger(DriveTrigger.BeginOTF);
    }
  }

  /**
   * checks if lineup is within 0.01 of tag + offsets
   *
   * @return true if error is small enoguh
   */
  public boolean lineupFinished() {
    boolean switchedSides = checkForSideSwitch();
    boolean latestObservationIsValid = latestObservation != null && latestObservation.isValid();
    Logger.recordOutput("Drive/lineup/latestObservationIsValid", latestObservationIsValid);

    DistanceToTag observation;
    if (latestObservationIsValid) {
      observation = latestObservation;
    } else {
      observation = updateDistanceFromCachedPose();
    }

    boolean rotationCorrect =
        Math.abs(drive.getRotation().getRadians() - getRotationForReefSide().getRadians())
            < JsonConstants.drivetrainConstants.lineupRotationMarginRadians;
    boolean alongTrackCorrect =
        observation.alongTrackDistance()
            < JsonConstants.drivetrainConstants.lineupAlongTrackThresholdMeters;
    boolean crossTrackCorrect =
        Math.abs(observation.crossTrackDistance())
            < JsonConstants.drivetrainConstants.lineupCrossTrackThresholdMeters;
    boolean vyLowEnough =
        Math.abs(drive.getChassisSpeeds().vyMetersPerSecond)
            < JsonConstants.drivetrainConstants.lineupVyThresholdMetersPerSecond;

    Logger.recordOutput("Drive/lineup/hadObservationYet", hadObservationYet);
    Logger.recordOutput("Drive/lineup/sideSwitched", switchedSides);
    Logger.recordOutput("Drive/lineup/rotationCorrect", rotationCorrect);
    Logger.recordOutput("Drive/lineup/alongTrackCorrect", alongTrackCorrect);
    Logger.recordOutput("Drive/lineup/crossTrackCorrect", crossTrackCorrect);
    Logger.recordOutput("Drive/lineup/vyLowEnough", vyLowEnough);

    return (JsonConstants.drivetrainConstants.allowLineupFinishWithCachedObservation
            || latestObservationIsValid)
        && hadObservationYet
        && rotationCorrect
        && alongTrackCorrect
        && crossTrackCorrect
        && vyLowEnough
        && !switchedSides;
  }

  public void periodic() {
    if (StrategyManager.getInstance().getAutonomyMode() == AutonomyMode.Manual) {
      drive.fireTrigger(DriveTrigger.CancelLineup);
    }

    // run test through here if in test mode
    if (DriverStation.isTest()) {
      this.testPeriodic();
    }

    // handles drive state due to reef location change
    checkForReefUpdate();

    this.LineupWithReefLocation();

    drive.setDriveLinedUp(lineupFinished());
    Logger.recordOutput("drive/lineup/finished", lineupFinished());
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

    // slow down when near reef
    if (Math.abs(alongTrackDistance)
        < JsonConstants.drivetrainConstants.lineupAlongTrackSlowDownDistance) {
      multiplier = JsonConstants.drivetrainConstants.lineupAlongTrackSlowDownMultiplier;
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
    // check to add or subtract offset error
    int signOfError = cameraIndex == JsonConstants.visionConstants.FrontLeftCameraIndex ? -1 : 1;
    // distance between cameras (add or subtract so we still lock on to correct side)
    double offsetErrorCorrection =
        checkForSideSwitch()
            ? 0
            : JsonConstants.visionConstants.FrontLeftTransform.getY()
                - JsonConstants.visionConstants.FrontRightTransform.getY();
    DistanceToTag observationOtherCamera =
        alignmentSupplier.get(
            tagId,
            otherCameraIndex,
            ReefLineupUtil.getCrossTrackOffset(otherCameraIndex)
                + (signOfError * offsetErrorCorrection),
            JsonConstants.drivetrainConstants.driveAlongTrackOffset);
    return observationOtherCamera;
  }

  /**
   * calculates our expected distance errror based on global odometry fall back to this when there
   * is no new observation or it isnt valid
   *
   * @return a DistanceToTag of the new expected distances needed to reach setpoint
   */
  public DistanceToTag updateDistanceFromCachedPose() {

    if (poseAtLastObservation == null || latestObservation == null) {
      return new DistanceToTag(0, 0, false);
    }
    Translation2d adjustment =
        drive
            .getPose()
            .minus(poseAtLastObservation)
            .getTranslation()
            .rotateBy(getRotationForReefSide());

    Logger.recordOutput("Drive/Lineup/Adjustment", adjustment);

    DistanceToTag adjustedObservation =
        new DistanceToTag(
            latestObservation.crossTrackDistance() + adjustment.getY(),
            latestObservation.alongTrackDistance() - adjustment.getX(),
            true);
    return adjustedObservation;
  }

  /** take over goal speeds to align to reef exactly */
  public void LineupWithReefLocation() {
    int tagId = ReefLineupUtil.getTagIdForReef(drive);
    int cameraIndex = ReefLineupUtil.getCameraIndexForLineup(drive);

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
            ReefLineupUtil.getCrossTrackOffset(cameraIndex),
            JsonConstants.drivetrainConstants.driveAlongTrackOffset);

    DistanceToTag otherCameraObs = tryOtherCamera(alignmentSupplier, tagId, cameraIndex);

    Logger.recordOutput("Drive/Lineup/newObservationValid", observation.isValid());

    Logger.recordOutput("Drive/Lineup/usingOtherCamera", false);

    if (observation.isValid()) {
      usingOtherCamera = false;
      driveCrossTrackOtherCameraLineupController.reset();
      latestObservation = observation;

      poseAtLastObservation = drive.getPose();

      hadObservationYet = true;
      observationAge = 0;
    } else if (otherCameraObs != null && otherCameraObs.isValid()) {
      // check if the other camera has observation (maybe we switched to other pole or camera got
      // unplugged)
      usingOtherCamera = true;
      driveCrossTrackLineupController.reset();
      latestObservation = otherCameraObs;
      observation = otherCameraObs;
      observationAge = 0;
      Logger.recordOutput("Drive/Lineup/usingOtherCamera", true);
    } else {
      observation = updateDistanceFromCachedPose();
      if (!observation.isValid()) {
        // go back to otf?
      }
    }

    Logger.recordOutput(
        "Drive/Lineup/FinalPose",
        drive
            .getPose()
            .plus(
                new Transform2d(
                    new Translation2d(
                            observation.alongTrackDistance(), observation.crossTrackDistance())
                        .rotateBy(drive.getRotation()),
                    Rotation2d.kZero)));

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
              * getAlongTrackVelocity(observation.alongTrackDistance()); // Maybe use filtered?
      double vy = driveCrossTrackLineupController.calculate(observation.crossTrackDistance());
      double omega =
          rotationController.calculate(
              drive.getRotation().getRadians(), this.getRotationForReefSide().getRadians());

      if (usingOtherCamera) {
        vy = driveCrossTrackOtherCameraLineupController.calculate(observation.crossTrackDistance());
      }

      drive.setGoalSpeeds(new ChassisSpeeds(vx, vy, omega), false);
    }
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
