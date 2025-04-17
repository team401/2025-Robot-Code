package frc.robot.commands.strategies;

import edu.wpi.first.math.geometry.Translation2d;
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

  public AutoDriveToLocation(
      Drive drive, DesiredLocation desiredLocation, boolean shouldLinearDriveSlowly) {
    this.drive = drive;
    this.desiredLocation = desiredLocation;

    this.shouldLinearDriveSlowly = shouldLinearDriveSlowly;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    System.out.println("AutoDriveToLocation initializing!");
    if (drive != null) {
      drive.setDriveLinedUp(false);
      drive.setGoToIntake(false);
      drive.setShouldLinearDriveSlowly(shouldLinearDriveSlowly);
      drive.setDesiredLocation(desiredLocation);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.fireTrigger(DriveTrigger.BeginLinear);
    }
  }

  public void end(boolean interrupted) {
    System.out.println("AutoDriveToLine finished! Interrupted: " + interrupted);
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelLinear);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.setShouldLinearDriveSlowly(false);
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (drive == null) {
      Logger.recordOutput("AUtoDriveToLocation/driveFinished", true);
      return true;
    }

    Translation2d goalTranslation =
        DesiredLocationUtil.findGoalPoseFromDesiredLocation(
                drive.getDesiredLocation(), drive.isAllianceRed())
            .getTranslation();

    // isDriveCloseToFinalLineupPose() is untrustworthy (probably due to stale state) so it's
    // necessary to compute this here
    boolean isDriveClose =
        drive.getPose().getTranslation().getDistance(goalTranslation)
            < JsonConstants.drivetrainConstants.kDriveToPointFinishMargin;

    Logger.recordOutput("AUtoDriveToLocation/driveFinished", isDriveClose);

    return isDriveClose;
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
