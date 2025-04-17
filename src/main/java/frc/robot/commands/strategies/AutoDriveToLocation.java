package frc.robot.commands.strategies;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.DesiredLocationUtil;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import org.littletonrobotics.junction.Logger;

/** This command linear-drives to a specified DesiredLocation */
public class AutoDriveToLocation extends Command {
  private Drive drive;

  private final DesiredLocation desiredLocation;
  private final boolean shouldLinearDriveSlowly;

  // Keep track of how many times this command has been scheduled to help in debugging
  private static int id = 0;

  private int currentId = id;

  public AutoDriveToLocation(
      Drive drive, DesiredLocation desiredLocation, boolean shouldLinearDriveSlowly) {
    this.drive = drive;
    this.desiredLocation = desiredLocation;

    this.shouldLinearDriveSlowly = shouldLinearDriveSlowly;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    currentId = id;
    id++;

    System.out.println(
        "AutoDriveToLocation initializing! ID: "
            + currentId
            + " DesiredLocation: "
            + desiredLocation);
    if (drive != null) {
      drive.setDriveLinedUp(false);
      if (drive.isLocationScoring(desiredLocation)) {
        drive.setGoToIntake(false);
        drive.setDesiredLocation(desiredLocation);
      } else {
        drive.setGoToIntake(true);
        drive.setDesiredIntakeLocation(desiredLocation);
      }
      drive.setShouldLinearDriveSlowly(shouldLinearDriveSlowly);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.fireTrigger(DriveTrigger.BeginLinear);
    }
  }

  public void execute() {
    if (drive != null && !isReadyForNextAction() && !drive.isLinearDriving()) {
      drive.setDriveLinedUp(false);
      if (drive.isLocationScoring(desiredLocation)) {
        drive.setGoToIntake(false);
        drive.setDesiredLocation(desiredLocation);
      } else {
        drive.setGoToIntake(true);
        drive.setDesiredIntakeLocation(desiredLocation);
      }
      drive.setShouldLinearDriveSlowly(shouldLinearDriveSlowly);
      drive.setDesiredLocation(desiredLocation);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.fireTrigger(DriveTrigger.BeginLinear);

      System.out.println("AutoDriveToLine " + currentId + " re-entered linear drive!");
    }
  }

  public void end(boolean interrupted) {
    System.out.println(
        "AutoDriveToLine ID "
            + currentId
            + " (DL: "
            + desiredLocation
            + ") finished! Interrupted: "
            + interrupted);
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelLinear);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.setShouldLinearDriveSlowly(false);
    } else {
      System.out.println("AutoDriveToLine had null drive reference");
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (drive == null) {
      Logger.recordOutput("AutoDriveToLocation/driveFinished", true);
      return true;
    }

    Pose2d goalPose =
        DesiredLocationUtil.findGoalPoseFromDesiredLocation(desiredLocation, drive.isAllianceRed());

    // isDriveCloseToFinalLineupPose() is untrustworthy (probably due to stale state) so it's
    // necessary to compute this here
    boolean isDriveClose =
        drive.getPose().getTranslation().getDistance(goalPose.getTranslation())
            < JsonConstants.drivetrainConstants.kDriveToPointFinishMargin;

    Logger.recordOutput("AutoDriveToLocation/goalPose", goalPose);
    Logger.recordOutput(
        "AutoDriveToLocation/finishMargin",
        JsonConstants.drivetrainConstants.kDriveToPointFinishMargin);
    Logger.recordOutput("AutoDriveToLocation/driveFinished", isDriveClose);

    if (isDriveClose) {
      System.out.println("AutoDriveToLocation isDriveClose was true:");
      System.out.println("ID " + currentId);
      System.out.println("DL " + desiredLocation);
      System.out.println("goalPose " + goalPose);
      System.out.println("drive pose " + drive.getPose());
    }
    return isDriveClose;
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
