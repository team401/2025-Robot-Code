package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import org.littletonrobotics.junction.Logger;

/** This command slowly drives to the auto line */
public class AutoDriveToLine extends Command {
  private Drive drive;

  private DesiredLocation desiredLocation;

  public AutoDriveToLine(Drive drive, DesiredLocation desiredLocation) {
    this.drive = drive;
    this.desiredLocation = desiredLocation;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    if (drive != null) {
      drive.setDriveLinedUp(false);
      drive.setGoToIntake(false);
      drive.setShouldLinearDriveSlowly(true);
      drive.setDesiredLocation(desiredLocation);
      drive.fireTrigger(DriveTrigger.BeginLinear);
    }
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      drive.setShouldLinearDriveSlowly(false);
      System.out.println("AutoDriveToLine canceled lineup!");
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
