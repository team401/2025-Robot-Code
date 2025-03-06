package frc.robot.subsystems.drive.states;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.path.PathConstraints;
import coppercore.controls.state_machine.state.PeriodicStateInterface;
import coppercore.controls.state_machine.transition.Transition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
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
    if (PathfindingCommand.warmupCommand().isScheduled()) {
      PathfindingCommand.warmupCommand().cancel();
    }
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
      case Reef1:
      case Algae0:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef01Translation,
                JsonConstants.redFieldLocations.redReef01Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef01Translation,
                JsonConstants.blueFieldLocations.blueReef01Rotation);
      case Reef2:
      case Reef3:
      case Algae1:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef23Translation,
                JsonConstants.redFieldLocations.redReef23Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef23Translation,
                JsonConstants.blueFieldLocations.blueReef23Rotation);
      case Reef4:
      case Reef5:
      case Algae2:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef45Translation,
                JsonConstants.redFieldLocations.redReef45Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef45Translation,
                JsonConstants.blueFieldLocations.blueReef45Rotation);
      case Reef6:
      case Reef7:
      case Algae3:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef67Translation,
                JsonConstants.redFieldLocations.redReef67Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef67Translation,
                JsonConstants.blueFieldLocations.blueReef67Rotation);
      case Reef8:
      case Reef9:
      case Algae4:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef89Translation,
                JsonConstants.redFieldLocations.redReef89Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef89Translation,
                JsonConstants.blueFieldLocations.blueReef89Rotation);
      case Reef10:
      case Reef11:
      case Algae5:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef1011Translation,
                JsonConstants.redFieldLocations.redReef1011Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef1011Translation,
                JsonConstants.blueFieldLocations.blueReef1011Rotation);
      case Processor:
        return new Pose2d();
      case Net:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redNetTranslation,
                JsonConstants.redFieldLocations.redNetRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueNetTranslation,
                JsonConstants.blueFieldLocations.blueNetRotation);
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

    PathConstraints constraints;
    if (drive.getDesiredLocation() == DesiredLocation.Net) {
      // Create the constraints to use while pathfinding
      constraints =
          new PathConstraints(
              JsonConstants.drivetrainConstants.OTFSlowLinearVelocity,
              JsonConstants.drivetrainConstants.OTFSlowLinearAccel,
              JsonConstants.drivetrainConstants.OTFSlowAngularVelocity,
              JsonConstants.drivetrainConstants.OTFSlowAngularAccel);
    } else {
      // Create the constraints to use while pathfinding
      constraints =
          new PathConstraints(
              JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
              JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
              JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
              JsonConstants.drivetrainConstants.OTFMaxAngularAccel);
    }

    if (drive.isGoingToIntake()) {
      constraints =
          new PathConstraints(
              JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
              JsonConstants.drivetrainConstants.OTFMaxLinearAccel - 1,
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
      ScoringSubsystem.getInstance().fireTrigger(ScoringTrigger.StartWarmup);
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
