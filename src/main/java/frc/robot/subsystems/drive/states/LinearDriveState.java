package frc.robot.subsystems.drive.states;

import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.controller.PIDController;
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
import frc.robot.subsystems.drive.DesiredLocationUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
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

  private double linearDriveErrorMargin = 0.25;

  private ProfiledPIDController driveController =
      new ProfiledPIDController(
          kDriveToPointTranslationP,
          kDriveToPointTranslationI,
          kDriveToPointTranslationD,
          new TrapezoidProfile.Constraints(
              kDriveTranslationMaxVelocity, kDriveTranslationMaxAcceleration));

  private PIDController headingController =
      new PIDController(kDriveToPointHeadingP, kDriveToPointHeadingI, kDriveToPointHeadingD);

  public LinearDriveState(Drive drive) {
    this.drive = drive;
  }

  private boolean hasRunPhase1 = false;
  private boolean hasEnteredPhase2 = false;

  public void onEntry(Transition transition) {
    hasRunPhase1 = false;
    hasEnteredPhase2 = false;

    goalPose = findLinearDriveFromDesiredLocation(drive);

    Pose2d currentPose = drive.getPose();

    Pose2d phase1Pose = findPhase1Pose(goalPose);

    TrapezoidProfile.Constraints driveConstraints =
        drive.shouldLinearDriveSlowly()
            ? new TrapezoidProfile.Constraints(
                JsonConstants.drivetrainConstants.kDriveTranslationMaxVelocitySlow,
                JsonConstants.drivetrainConstants.kDriveTranslationMaxAccelerationSlow)
            : new TrapezoidProfile.Constraints(
                JsonConstants.drivetrainConstants.kDriveTranslationMaxVelocity,
                JsonConstants.drivetrainConstants.kDriveTranslationMaxAcceleration);

    driveController =
        new ProfiledPIDController(
            JsonConstants.drivetrainConstants.kDriveToPointTranslationP,
            JsonConstants.drivetrainConstants.kDriveToPointTranslationI,
            JsonConstants.drivetrainConstants.kDriveToPointTranslationD,
            driveConstraints);

    double distanceToGoal = currentPose.getTranslation().getDistance(phase1Pose.getTranslation());
    driveController.reset(new State(distanceToGoal, 0.0));

    headingController =
        new PIDController(
            JsonConstants.drivetrainConstants.kDriveToPointHeadingP,
            JsonConstants.drivetrainConstants.kDriveToPointHeadingI,
            JsonConstants.drivetrainConstants.kDriveToPointHeadingD);

    headingController.reset();
    headingController.enableContinuousInput(-Math.PI, Math.PI);

    // Using the lineup margin as the linear drive margin is totally wrong:
    linearDriveErrorMargin = JsonConstants.drivetrainConstants.kDriveToPointFinishMargin;

    // drive.enableReefCenterAlignment();

    System.out.println(
        "Entered LinearDriveState with desired location " + drive.getDesiredLocation());
  }

  public void onExit(Transition transition) {
    // Disable reef center alignment in case we stop linear driving before reaching phase 2
    // drive.disableReefCenterAlignment();
  }

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public static Pose2d findLinearDriveFromDesiredLocation(Drive driveInput) {
    return DesiredLocationUtil.findGoalPoseFromDesiredLocation(
        driveInput.getDesiredLocation(), driveInput.isAllianceRed());
  }

  private double getAngleBetweenVectors(Translation2d u, Translation2d v) {
    double u_norm = u.getNorm();
    double v_norm = v.getNorm();
    double dot_product = u.toVector().dot(v.toVector());

    // Avoid cases that break acos() and return 0.0 in these cases.
    if (u_norm * v_norm < 1e-10 || dot_product > (u_norm * v_norm)) {
      return 0.0;
    }
    return Math.acos(dot_product / (u_norm * v_norm));
  }

  private double driveVelocityScalar = 0.0;

  private Pose2d findPhase1Pose(Pose2d phase2Pose) {
    // Project the goal pose backwards to find the phase 1 goal pose.
    double phase1OffsetX =
        JsonConstants.drivetrainConstants.kDriveToPointPhase2Distance
            * Math.cos(phase2Pose.getRotation().getRadians() + Math.PI);
    double phase1OffsetY =
        JsonConstants.drivetrainConstants.kDriveToPointPhase2Distance
            * Math.sin(phase2Pose.getRotation().getRadians() + Math.PI);
    Pose2d phase1Pose =
        new Pose2d(
            phase2Pose.getX() + phase1OffsetX,
            phase2Pose.getY() + phase1OffsetY,
            phase2Pose.getRotation());

    return phase1Pose;
  }

  @Override
  public void periodic() {
    Pose2d currentPose = drive.getPose();

    Pose2d phase1Pose = findPhase1Pose(goalPose);

    // Get distances to goal positions.
    double distanceToGoal = currentPose.getTranslation().getDistance(goalPose.getTranslation());
    double distanceToPhase1Pose =
        currentPose.getTranslation().getDistance(phase1Pose.getTranslation());

    // Find if the robot is within the slice area to transition to phase 2.
    Translation2d phase1PoseToGoal = goalPose.getTranslation().minus(phase1Pose.getTranslation());
    Translation2d robotToGoal = goalPose.getTranslation().minus(currentPose.getTranslation());
    double angleToTarget = getAngleBetweenVectors(robotToGoal, phase1PoseToGoal);

    // Determine which pose to aim at. However, always use the distance to the final pose to compute
    // the target speed.
    Pose2d currentGoalPose = phase1Pose;
    // Use whichever distance we're going to so that we slow down on the way to phase 1 pose
    double distanceToCurrentGoal = distanceToPhase1Pose;
    double endVelocityGoal = 0.0; // Try to reach 0 velocity in phase 1
    if (hasEnteredPhase2
        || distanceToGoal < JsonConstants.drivetrainConstants.kDriveToPointPhase2Distance
        || Math.abs(angleToTarget) < JsonConstants.drivetrainConstants.kDriveToPointPhase2Angle) {
      currentGoalPose = goalPose;

      // Slow linear drive should end at zero, normal linear drive should end at correct end speed
      if (!drive.shouldLinearDriveSlowly()) {
        endVelocityGoal = JsonConstants.drivetrainConstants.kDriveToPointEndVelocity;
      }

      if (!hasEnteredPhase2) {
        hasEnteredPhase2 = true;

        if (hasRunPhase1) {
          // Bumplessing
          State lastState = driveController.getSetpoint();
          double lastVelocity = lastState.velocity;
          double lastError = distanceToCurrentGoal - lastState.position;
          double adjustedPosition = distanceToGoal - lastError;
          driveController.reset(new State(adjustedPosition, lastVelocity));

          System.out.println("Switched to phase 2 with bumplessing");
        } else {
          driveController.reset(new State(distanceToGoal, 0.0));

          System.out.println("Switched to phase 2 instantly");
        }

        // drive.disableReefCenterAlignment();
      }

      // This has to be here because distanceToCurrentGoal is used for PID reset adjustment meme
      distanceToCurrentGoal = distanceToGoal;
    } else {
      hasRunPhase1 = true;
    }

    if (distanceToGoal < JsonConstants.drivetrainConstants.otfWarmupDistance
        && ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartEarlyWarmup);
    } else if (distanceToGoal < JsonConstants.drivetrainConstants.otfFarWarmupDistance
        && ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartFarWarmup);
    }

    // We only exit this state when the phase 2 pose has been achieved.
    if (distanceToGoal < linearDriveErrorMargin) {
      Logger.recordOutput("DriveToPoint/DriveDistance", distanceToGoal);
      System.out.println("Linear drive close to goal (distance " + distanceToGoal + ")");
      if (drive.isDesiredLocationReef()
          && (ScoringSubsystem.getInstance() == null
              || ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral)) {
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

    driveVelocityScalar =
        driveController.calculate(distanceToCurrentGoal, new State(0.0, endVelocityGoal));
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

    Logger.recordOutput("DriveToPoint/HasRunPhase1", hasRunPhase1);
    Logger.recordOutput("DriveToPoint/HasEnteredPhase2", hasEnteredPhase2);
    Logger.recordOutput("DriveToPoint/AngleToTarget", angleToTarget);
    Logger.recordOutput("DriveToPoint/Phase1Pose", phase1Pose);
    Logger.recordOutput("DriveToPoint/Phase2Pose", goalPose);
    Logger.recordOutput("DriveToPoint/TargetPose", currentGoalPose);
    Logger.recordOutput("DriveToPoint/PhaseDriveDistance", distanceToCurrentGoal);
    Logger.recordOutput("DriveToPoint/DriveDistance", distanceToGoal);
    Logger.recordOutput("DriveToPoint/HeadingError", headingError);
    Logger.recordOutput("DriveToPoint/setpoint/position", driveController.getSetpoint().position);
    Logger.recordOutput("DriveToPoint/setpoint/velocity", driveController.getSetpoint().velocity);

    Logger.recordOutput("DriveToPoint/DriveVelocityScalar", driveVelocityScalar);
    Logger.recordOutput("DriveToPoint/HeadingVelocity", headingVelocity);
    Logger.recordOutput("DriveToPoint/DriveVelocityX", driveVelocity.getX());
    Logger.recordOutput("DriveToPoint/DriveVelocityY", driveVelocity.getY());

    Logger.recordOutput("DriveToPoint/DriveSpeeds", speeds);
  }
}
