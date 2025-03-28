package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import frc.robot.constants.JsonConstants;
import frc.robot.constants.subsystems.DrivetrainConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import org.littletonrobotics.junction.Logger;

public class LinearDriveState implements PeriodicStateInterface {
  private Drive drive;

  private Pose2d goalPose = null;

  private double kDriveToPointTranslationP =
      DrivetrainConstants.synced.getObject().kDriveToPointTranslationP;
  private double kDriveToPointTranslationI =
      DrivetrainConstants.synced.getObject().kDriveToPointTranslationI;
  private double kDriveToPointTranslationD =
      DrivetrainConstants.synced.getObject().kDriveToPointTranslationD;
  private double kDriveTranslationMaxVelocity =
      DrivetrainConstants.synced.getObject().kDriveTranslationMaxVelocity;
  private double kDriveTranslationMaxAcceleration =
      DrivetrainConstants.synced.getObject().kDriveTranslationMaxAcceleration;

  private double kDriveToPointHeadingP =
      DrivetrainConstants.synced.getObject().kDriveToPointHeadingP;
  private double kDriveToPointHeadingI =
      DrivetrainConstants.synced.getObject().kDriveToPointHeadingI;
  private double kDriveToPointHeadingD =
      DrivetrainConstants.synced.getObject().kDriveToPointHeadingD;
  private double kDriveHeadingMaxVelocity =
      DrivetrainConstants.synced.getObject().kDriveHeadingMaxVelocity;
  private double kDriveHeadingMaxAcceleration =
      DrivetrainConstants.synced.getObject().kDriveHeadingMaxAcceleration;

  private double lineupErrorMargin = 0.05;

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

  public LinearDriveState(Drive drive) {
    this.drive = drive;
  }

  public void onEntry(Transition transition) {
    goalPose = findLinearDriveFromDesiredLocation(drive);

    Pose2d currentPose = drive.getPose();

    driveController =
        new ProfiledPIDController(
            JsonConstants.drivetrainConstants.kDriveToPointTranslationP,
            JsonConstants.drivetrainConstants.kDriveToPointTranslationI,
            JsonConstants.drivetrainConstants.kDriveToPointTranslationD,
            new TrapezoidProfile.Constraints(
                JsonConstants.drivetrainConstants.kDriveTranslationMaxVelocity,
                JsonConstants.drivetrainConstants.kDriveTranslationMaxAcceleration));

    double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    driveController.reset(new State(distanceToGoal, 0.0));

    headingController =
        new ProfiledPIDController(
            JsonConstants.drivetrainConstants.kDriveToPointHeadingP,
            JsonConstants.drivetrainConstants.kDriveToPointHeadingI,
            JsonConstants.drivetrainConstants.kDriveToPointHeadingD,
            new TrapezoidProfile.Constraints(
                JsonConstants.drivetrainConstants.kDriveHeadingMaxVelocity,
                JsonConstants.drivetrainConstants.kDriveHeadingMaxAcceleration));

    headingController.enableContinuousInput(0.0, Math.PI * 2);
    headingController.reset(currentPose.getRotation().getRadians());

    lineupErrorMargin = JsonConstants.drivetrainConstants.lineupErrorMargin;
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

  final double kPhase2Distance = 1.0;
  final double kPhase2Angle = Math.toRadians(45.0);
  final double kPhase1EndSpeed = 2.0;

  private double getAngleBetweenVectors(Translation2d u, Translation2d v) {
    double u_norm = u.getNorm();
    double v_norm = v.getNorm();
    double dot_product = u.getX() * v.getX() + u.getY() * v.getY();

    // Avoid cases that break acos() and return 0.0 in these cases.
    if (u_norm * v_norm < 1e-10 || dot_product > (u_norm * v_norm)) {
      return 0.0;
    }
    return Math.acos(dot_product / (u_norm * v_norm));
  }

  @Override
  public void periodic() {
    Pose2d currentPose = drive.getPose();

    // Project the goal pose backwards to find the phase 1 goal pose.
    double phase1OffsetX =
        kPhase2Distance * Math.cos(goalPose.getRotation().getRadians() + Math.PI);
    double phase1OffsetY =
        kPhase2Distance * Math.sin(goalPose.getRotation().getRadians() + Math.PI);
    Pose2d phase1Pose =
        new Pose2d(
            goalPose.getX() + phase1OffsetX,
            goalPose.getY() + phase1OffsetY,
            goalPose.getRotation());

    // Get distances to goal positions.
    double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    // double distanceToPhase1Pose =
    //     currentPose.getTranslation().getDistance(phase1Pose.getTranslation());

    // Find if the robot is within the slice area to transition to phase 2.
    Translation2d phase1PoseToGoal = goalPose.minus(phase1Pose).getTranslation();
    Translation2d robotToGoal = goalPose.minus(currentPose).getTranslation();
    double angleToTarget = getAngleBetweenVectors(robotToGoal, phase1PoseToGoal);

    // Determine which pose to aim at. However, always use the distance to the final pose to compute
    // the target speed.
    Pose2d currentGoalPose = phase1Pose;
    if (distanceToGoal < kPhase2Distance || Math.abs(angleToTarget) < kPhase2Angle) {
      currentGoalPose = goalPose;
    }

    // We only exit this state when the phase 2 pose has been achieved.
    if (distanceToGoal < lineupErrorMargin) {
      if (drive.isDesiredLocationReef()) {
        drive.fireTrigger(DriveTrigger.BeginLineup);
      } else {
        drive.fireTrigger(DriveTrigger.CancelLinear);
      }
      return;
    }

    // HEADING
    double headingError = currentPose.getRotation().minus(goalPose.getRotation()).getRadians();
    double headingVelocity =
        headingController.calculate(
            currentPose.getRotation().getRadians(), goalPose.getRotation().getRadians());

    double driveVelocityScalar =
        driveController.calculate(
            distanceToGoal,
            new State(0.0, JsonConstants.drivetrainConstants.kDriveToPointEndVelocity));
    Translation2d driveVelocity =
        new Pose2d(
                0.0,
                0.0,
                currentPose.getTranslation().minus(currentGoalPose.getTranslation()).getAngle())
            .transformBy(new Transform2d(driveVelocityScalar, 0.0, new Rotation2d()))
            .getTranslation();

    ChassisSpeeds speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            driveVelocity.getX(), driveVelocity.getY(), headingVelocity, currentPose.getRotation());
    drive.setGoalSpeeds(speeds, false);

    Logger.recordOutput("DriveToPoint/AngleToTarget", angleToTarget);
    Logger.recordOutput("DriveToPoint/Phase1Pose", phase1Pose);
    Logger.recordOutput("DriveToPoint/Phase2Pose", goalPose);
    Logger.recordOutput("DriveToPoint/TargetPose", currentGoalPose);
    Logger.recordOutput("DriveToPoint/DriveDistance", distanceToGoal);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);

    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);
  }
}
