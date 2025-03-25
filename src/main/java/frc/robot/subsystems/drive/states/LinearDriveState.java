package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import org.littletonrobotics.junction.Logger;

public class LinearDriveState implements PeriodicStateInterface {
  private Drive drive;

  private Pose2d goalPose = null;

  private double kDriveToPointTranslationP = 10;
  private double kDriveToPointTranslationI = 0;
  private double kDriveToPointTranslationD = 0;
  private double kDriveTranslationMaxVelocity = 5;
  private double kDriveTranslationMaxAcceleration = 5;
  private double kPositionTolerance = 0.005;
  private double kVelocityTolerance = 0.005;

  private double kDriveToPointHeadingP = 10;
  private double kDriveToPointHeadingI = 0;
  private double kDriveToPointHeadingD = 0;
  private double kDriveHeadingMaxVelocity = 5;
  private double kDriveHeadingMaxAcceleration = 5;
  private double kAngleTolerance = 0.005;
  private double kAngularVelocityTolerance = 0.005;

  private ProfiledPIDController driveController =
      new ProfiledPIDController(
          kDriveToPointTranslationP,
          kDriveToPointTranslationI,
          kDriveToPointTranslationD,
          new TrapezoidProfile.Constraints(
              kDriveTranslationMaxVelocity, kDriveTranslationMaxAcceleration));

  private ProfiledPIDController headingController =
      new ProfiledPIDController(
          kDriveToPointHeadingP,
          kDriveToPointHeadingI,
          kDriveToPointHeadingD,
          new TrapezoidProfile.Constraints(kDriveHeadingMaxVelocity, kDriveHeadingMaxAcceleration));

  private double m_ffMinRadius = 0.35, m_ffMaxRadius = 0.8;

  public LinearDriveState(Drive drive) {
    this.drive = drive;
    driveController.setTolerance(kPositionTolerance, kVelocityTolerance);
    driveController.setTolerance(kAngleTolerance, kAngularVelocityTolerance);
  }

  public void onEntry(Transition transition) {
    goalPose = findLinearDriveFromDesiredLocation(drive);
  }

  public void onExit(Transition transition) {}

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public static Pose2d findLinearDriveFromDesiredLocation(Drive driveInput) {
    switch (driveInput.getDesiredLocation()) {
        // NOTE: pairs of reef sides (ie 0 and 1) will have the same otf pose (approximately 0.5-1
        // meter away from center of tag)
      case Reef0:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF0Translation,
                JsonConstants.redFieldLocations.redReefOTF0Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF0Translation,
                JsonConstants.blueFieldLocations.blueReefOTF0Rotation);
      case Reef1:
      case Algae0:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF1Translation,
                JsonConstants.redFieldLocations.redReefOTF1Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF1Translation,
                JsonConstants.blueFieldLocations.blueReefOTF1Rotation);
      case Reef2:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF2Translation,
                JsonConstants.redFieldLocations.redReefOTF2Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF2Translation,
                JsonConstants.blueFieldLocations.blueReefOTF2Rotation);
      case Reef3:
      case Algae1:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF3Translation,
                JsonConstants.redFieldLocations.redReefOTF3Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF3Translation,
                JsonConstants.blueFieldLocations.blueReefOTF3Rotation);
      case Reef4:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF4Translation,
                JsonConstants.redFieldLocations.redReefOTF4Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF4Translation,
                JsonConstants.blueFieldLocations.blueReefOTF4Rotation);
      case Reef5:
      case Algae2:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF5Translation,
                JsonConstants.redFieldLocations.redReefOTF5Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF5Translation,
                JsonConstants.blueFieldLocations.blueReefOTF5Rotation);
      case Reef6:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF6Translation,
                JsonConstants.redFieldLocations.redReefOTF6Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF6Translation,
                JsonConstants.blueFieldLocations.blueReefOTF6Rotation);
      case Reef7:
      case Algae3:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF7Translation,
                JsonConstants.redFieldLocations.redReefOTF7Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF7Translation,
                JsonConstants.blueFieldLocations.blueReefOTF7Rotation);
      case Reef8:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF8Translation,
                JsonConstants.redFieldLocations.redReefOTF8Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF8Translation,
                JsonConstants.blueFieldLocations.blueReefOTF8Rotation);
      case Reef9:
      case Algae4:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF9Translation,
                JsonConstants.redFieldLocations.redReefOTF9Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF9Translation,
                JsonConstants.blueFieldLocations.blueReefOTF9Rotation);
      case Reef10:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF10Translation,
                JsonConstants.redFieldLocations.redReefOTF10Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF10Translation,
                JsonConstants.blueFieldLocations.blueReefOTF10Rotation);
      case Reef11:
      case Algae5:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF11Translation,
                JsonConstants.redFieldLocations.redReefOTF11Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF11Translation,
                JsonConstants.blueFieldLocations.blueReefOTF11Rotation);
      case CoralStationRight:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redCoralStationRightTranslation,
                JsonConstants.redFieldLocations.redCoralStationRightRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueCoralStationRightTranslation,
                JsonConstants.blueFieldLocations.blueCoralStationRightRotation);
      case CoralStationLeft:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redCoralStationLeftTranslation,
                JsonConstants.redFieldLocations.redCoralStationLeftRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueCoralStationLeftTranslation,
                JsonConstants.blueFieldLocations.blueCoralStationLeftRotation);
      default:
        return null;
    }
  }

  @Override
  public void periodic() {
    Pose2d currentPose = drive.getPose();

    double currentDistance = currentPose.getTranslation().getDistance(goalPose.getTranslation());

    if (withinRange(currentPose, goalPose, 0.05)) {
      drive.setGoalSpeeds(new ChassisSpeeds(), true);

      if (drive.isDesiredLocationReef()) {
        drive.fireTrigger(DriveTrigger.BeginLineup);
      } else {
        drive.fireTrigger(DriveTrigger.CancelLinear);
      }
      return;
    }

    // DRIVE
    double ffScaler =
        MathUtil.clamp(
            (currentDistance - m_ffMinRadius) / (m_ffMaxRadius - m_ffMinRadius), 0.0, 1.0);
    double driveVelocityScalar =
        driveController.getSetpoint().velocity * ffScaler
            + driveController.calculate(currentDistance, 0.0);
    if (currentDistance < driveController.getPositionTolerance()) {
      driveVelocityScalar = 0.0;
    }

    // HEADING
    double headingError = currentPose.getRotation().minus(goalPose.getRotation()).getRadians();
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());
    if (Math.abs(headingError) < headingController.getPositionTolerance()) {
      headingVelocity = 0.0;
    }

    Translation2d driveVelocity =
        new Pose2d(
                0.0, 0.0, currentPose.getTranslation().minus(goalPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();
    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), headingVelocity, currentPose.getRotation());
    drive.setGoalSpeeds(speeds, false);

    Logger.recordOutput("DriveToPoint/TargetPose", goalPose);
    Logger.recordOutput("DriveToPoint/DriveDistance", currentDistance);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);

    Logger.recordOutput("DriveToPoint/FFScaler", ffScaler);
    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);
  }

  public boolean withinRange(Pose2d a, Pose2d b, double error) {
    return Math.pow(a.getTranslation().getX() - b.getTranslation().getX(), 2)
                + Math.pow(a.getTranslation().getY() - b.getTranslation().getY(), 2)
            <= error * error
        && Math.abs(a.getRotation().getRadians() - b.getRotation().getRadians()) <= error;
  }
}
