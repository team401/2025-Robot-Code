package frc.robot.subsystems.drive.states;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

public class OTFState implements PeriodicStateInterface {
  private Drive drive;

  private Command driveToPose = null;

  private Pose2d otfPose = null;

  public OTFState(Drive drive) {
    this.drive = drive;
  }

  public void onEntry(Transition transition) {
    driveToPose = this.getDriveToPoseCommand();
    System.out.println(driveToPose == null);
    if (driveToPose == null) {
      drive.fireTrigger(DriveTrigger.CancelOTF);
    }

    System.out.println("running on entry correct");
    this.driveToPose.schedule();
  }

  public void onExit(Transition transition) {
    otfPose = null;
    System.out.println("Cancelling command");
    if (driveToPose != null) {
      this.driveToPose.cancel();
    }
  }

  /**
   * finds a pose to pathfind to based on desiredLocation enum
   *
   * @return a pose representing the corresponding scoring location
   */
  public static Pose2d findOTFPoseFromDesiredLocation(Drive driveInput) {
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
      case AutoLine:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redAutoLineTranslation,
                JsonConstants.redFieldLocations.redAutoLineRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueAutoLineTranslation,
                JsonConstants.blueFieldLocations.blueAutoLineRotation);
      case NetScore:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redNetScoreTranslation,
                JsonConstants.redFieldLocations.redNetScoreRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueNetScoreTranslation,
                JsonConstants.blueFieldLocations.blueNetScoreRotation);
      default:
        return null;
    }
  }

  /**
   * gets the path from current pose to the desired pose found from location
   *
   * @return command that drive can schedule to follow the path found
   */
  public Command getDriveToPoseCommand() {
    otfPose = findOTFPoseFromDesiredLocation(drive);

    if (otfPose == null) {
      return null;
    }

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
            JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
            JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
            JsonConstants.drivetrainConstants.OTFMaxAngularAccel);

    if (drive.isGoingToIntake()) {
      constraints =
          new PathConstraints(
              JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
              JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
              JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
              JsonConstants.drivetrainConstants.OTFMaxAngularAccel);
    }

    return AutoBuilder.pathfindToPose(
        otfPose,
        constraints,
        drive.isGoingToIntake() ? 0 : JsonConstants.drivetrainConstants.otfPoseEndingVelocity);
  }

  public void periodic() {
    // checks if location has changed (so path can be rescheduled)
    if (otfPose == null || !otfPose.equals(findOTFPoseFromDesiredLocation(drive))) {
      System.out.println("fail on entry");
      this.onEntry(null);
    }

    // finishes otf when we are 0.1 meters away
    // if (drive.isDriveCloseToFinalLineupPose()) {
    //   drive.fireTrigger(DriveTrigger.FinishOTF);
    // }

    if (drive.isDriveCloseForWarmup() && ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartEarlyWarmup);
    } else if (drive.isDriveCloseForFarWarmup() && ScoringSubsystem.getInstance() != null) {
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartFarWarmup);
    }

    if (driveToPose != null) {
      Logger.recordOutput("Drive/OTF/commandScheduled", driveToPose.isScheduled());
      Logger.recordOutput("Drive/OTF/commandFinished", driveToPose.isFinished());

      // reschedule command if its not actually close
      // if (driveToPose.isFinished() && !drive.isDriveCloseToFinalLineupPose()) {
      //   driveToPose = this.getDriveToPoseCommand();
      //   if (driveToPose == null) {
      //     drive.fireTrigger(DriveTrigger.CancelOTF);
      //   }
      //   this.driveToPose.schedule();
      //   // go to lineup if we want reef
      // } else
      if ((driveToPose == null || driveToPose.isFinished())
          && !drive.isDriveCloseToFinalLineupPose()) {
        this.driveToPose = getDriveToPoseCommand();
        if (driveToPose != null) {
          driveToPose.schedule();
        }
      } else if (driveToPose.isFinished() && drive.isDesiredLocationReef()) {
        drive.fireTrigger(DriveTrigger.BeginLineup);
        // go to joystick otherwise
      } else if (driveToPose.isFinished()) {
        drive.fireTrigger(DriveTrigger.CancelOTF);
        System.out.println("driveToPose isFinished canceled OTF!");
      }

      // if (drive.isDesiredLocationReef()) {
      //   // Check if we are close to a tag and can see the desired tag and go to OTF
      //   int tagId = ReefLineupUtil.getTagIdForReef(drive);
      //   int cameraIndex = ReefLineupUtil.getCameraIndexForLineup(drive);

      //   if (tagId == -1 || cameraIndex == -1) {
      //     // TODO: check if this might be false first time, but on another loop true
      //     // drive.fireTrigger(DriveTrigger.CancelLineup);
      //     return;
      //   }

      //   VisionAlignment alignmentSupplier = drive.getVisionAlignment();

      //   DistanceToTag observation =
      //       alignmentSupplier.get(
      //           tagId,
      //           cameraIndex,
      //           ReefLineupUtil.getCrossTrackOffset(cameraIndex),
      //           JsonConstants.drivetrainConstants.driveAlongTrackOffset);

      //   if (observation.isValid()
      //       && observation.alongTrackDistance()
      //           < JsonConstants.drivetrainConstants.otfVisionAlongTrackThreshold
      //       && Math.abs(observation.crossTrackDistance())
      //           < JsonConstants.drivetrainConstants.otfVisionCrossTrackThreshold) {
      //     drive.fireTrigger(DriveTrigger.BeginLineup);
      //   }
      // }
    }
  }
}
