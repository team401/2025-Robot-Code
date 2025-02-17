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
import frc.robot.subsystems.drive.Drive.DriveTrigger;
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
    if (driveToPose == null) {
      drive.fireTrigger(DriveTrigger.CancelOTF);
    }
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
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef01Translation,
                JsonConstants.redFieldLocations.redReef01Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef01Translation,
                JsonConstants.blueFieldLocations.blueReef01Rotation);
      case Reef2:
      case Reef3:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef23Translation,
                JsonConstants.redFieldLocations.redReef23Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef23Translation,
                JsonConstants.blueFieldLocations.blueReef23Rotation);
      case Reef4:
      case Reef5:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef45Translation,
                JsonConstants.redFieldLocations.redReef45Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef45Translation,
                JsonConstants.blueFieldLocations.blueReef45Rotation);
      case Reef6:
      case Reef7:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef67Translation,
                JsonConstants.redFieldLocations.redReef67Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef67Translation,
                JsonConstants.blueFieldLocations.blueReef67Rotation);
      case Reef8:
      case Reef9:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef89Translation,
                JsonConstants.redFieldLocations.redReef89Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef89Translation,
                JsonConstants.blueFieldLocations.blueReef89Rotation);
      case Reef10:
      case Reef11:
        return driveInput.isAllianceRed()
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReef1011Translation,
                JsonConstants.redFieldLocations.redReef1011Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReef1011Translation,
                JsonConstants.blueFieldLocations.blueReef1011Rotation);
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

    // Create the constraints to use while pathfinding
    PathConstraints constraints =
        new PathConstraints(
            JsonConstants.drivetrainConstants.OTFMaxLinearVelocity,
            JsonConstants.drivetrainConstants.OTFMaxLinearAccel,
            JsonConstants.drivetrainConstants.OTFMaxAngularVelocity,
            JsonConstants.drivetrainConstants.OTFMaxAngularAccel);

    return AutoBuilder.pathfindToPose(otfPose, constraints, 0.0);
  }

  public void periodic() {
    // checks if location has changed (so path can be rescheduled)
    if (otfPose == null || !otfPose.equals(findOTFPoseFromDesiredLocation(drive))) {
      this.onEntry(null);
    }

    // finishes otf when we are 0.1 meters away
    if (drive.isDriveCloseToFinalLineupPose()) {
      drive.fireTrigger(DriveTrigger.FinishOTF);
    }

    if (driveToPose != null) {
      Logger.recordOutput("Drive/OTF/commandScheduled", driveToPose.isScheduled());

      // this runs when we accidentally go into otf (too close to reef for final pose to be true)
      if (driveToPose.isFinished() && drive.isDesiredLocationReef()) {
        drive.fireTrigger(DriveTrigger.BeginLineup);
      } else if (driveToPose.isFinished()) {
        drive.fireTrigger(DriveTrigger.CancelOTF);
      }
    }
  }
}
