package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
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
    if (drive != null) {
      drive.setDriveLinedUp(false);
      drive.setGoToIntake(false);
      drive.setShouldLinearDriveSlowly(shouldLinearDriveSlowly);
      drive.setDesiredLocation(desiredLocation);
      drive.fireTrigger(DriveTrigger.BeginLinear);
    }
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelLinear);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.setShouldLinearDriveSlowly(false);
      System.out.println("AutoDriveToLine canceled lineup! Interrupted: " + interrupted);
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (drive != null) {
      Logger.recordOutput("AutoScore/driveFinished", drive.isDriveCloseToFinalLineupPose());
    } else {
      Logger.recordOutput("AutoScore/driveFinished", true);
    }

    return (drive == null || drive.isDriveCloseToFinalLineupPose());
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
