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
import frc.robot.StrategyManager;
import frc.robot.StrategyManager.AutonomyMode;
import frc.robot.TestModeManager;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.ModeConstants;
import frc.robot.constants.ModeConstants.Mode;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.drive.Drive.VisionAlignment;
import frc.robot.subsystems.drive.ReefLineupUtil;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.commands.PathfindingCommand;

public class LineupState implements PeriodicStateInterface {
  private Drive drive;

  private DistanceToTag latestObservation;
  private boolean hadObservationYet = false;
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

    if(PathfindingCommand.warmupCommand().isScheduled()) {
      PathfindingCommand.warmupCommand().cancel();
    }
    // Drive isn't done lining up
    drive.setDriveLinedUp(false);

    // cancel rotation lock on center
    drive.disableAlign();

    // dont rely on previous estimates
    latestObservation = null;
    observationAge = 0;

    hadObservationYet = false;
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
    boolean latestObservationExists = latestObservation != null;
    Logger.recordOutput("Drive/lineup/latestObservationExists", latestObservationExists);
    if (!latestObservationExists) {
      return false;
    }

    boolean rotationCorrect =
        Math.abs(drive.getRotation().getRadians() - getRotationForReefSide().getRadians())
            < JsonConstants.drivetrainConstants.lineupRotationMarginRadians;
    boolean alongTrackCorrect =
        latestObservation.alongTrackDistance()
            < JsonConstants.drivetrainConstants.lineupAlongTrackThresholdMeters;
    boolean crossTrackCorrect =
        Math.abs(latestObservation.crossTrackDistance())
            < JsonConstants.drivetrainConstants.lineupCrossTrackThresholdMeters;
    boolean vyLowEnough =
        Math.abs(drive.getChassisSpeeds().vyMetersPerSecond)
            < JsonConstants.drivetrainConstants.lineupVyThresholdMetersPerSecond;

    Logger.recordOutput("Drive/lineup/rotationCorrect", rotationCorrect);
    Logger.recordOutput("Drive/lineup/alongTrackCorrect", alongTrackCorrect);
    Logger.recordOutput("Drive/lineup/crossTrackCorrect", crossTrackCorrect);
    Logger.recordOutput("Drive/lineup/vyLowEnough", vyLowEnough);

    return latestObservationExists
        && hadObservationYet
        && rotationCorrect
        && alongTrackCorrect
        && crossTrackCorrect
        && vyLowEnough;
  }

  public void periodic() {
    if (StrategyManager.getInstance().getAutonomyMode() == AutonomyMode.Manual) {
      drive.fireTrigger(DriveTrigger.CancelLineup);
    }

    // run test through here if in test mode
    if (DriverStation.isTest()) {
      this.testPeriodic();
    }
    this.LineupWithReefLocation();

    drive.setDriveLinedUp(lineupFinished());
    Logger.recordOutput("lineup/finished", lineupFinished());
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
    otherCameraTried = true;
    DistanceToTag observationOtherCamera =
        alignmentSupplier.get(
            tagId,
            otherCameraIndex,
            ReefLineupUtil.getCrossTrackOffset(otherCameraIndex),
            JsonConstants.drivetrainConstants.driveAlongTrackOffset);
    if (observationOtherCamera.isValid()) {
      return observationOtherCamera;
    }

    return null;
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

    if (!observation.isValid()) {
      if (observationAge < 5) {
        if (latestObservation != null && latestObservation.isValid()) {
          observation = latestObservation;
        }
        observationAge++;
      } else if (observationAge > 5) {
        // cancel lineup if we havent seen a observation after five times
        drive.fireTrigger(DriveTrigger.CancelLineup);
      }
      // } else if (!otherCameraTried) {
      //   // check other camera (for when otf comes in from other side)
      //   DistanceToTag otherCameraObs = tryOtherCamera(alignmentSupplier, tagId, cameraIndex);

      //   if (otherCameraObs != null) {
      //     observation = otherCameraObs;
      //   }
      // }
    } else {
      latestObservation = observation;
      // begin warming up elevator/wrist when lineup starts
      if (ScoringSubsystem.getInstance() != null) {
        ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
      }
      hadObservationYet = true;
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
              * getAlongTrackVelocity(observation.alongTrackDistance()); // Maybe use filtered?
      double vy = driveCrossTrackLineupController.calculate(observation.crossTrackDistance());
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
