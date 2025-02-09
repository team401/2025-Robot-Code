package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class AutoIntake extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  private DesiredLocation intakeLocation;

  public AutoIntake(Drive drive, ScoringSubsystem scoring, DesiredLocation intakeLocation) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.intakeLocation = intakeLocation;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    if (drive != null) {
      drive.setDesiredIntakeLocation(intakeLocation);
      drive.setGoToIntake(true);
      drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    return (drive == null || drive.isDriveAlignmentFinished())
        && (scoringSubsystem == null || !scoringSubsystem.shouldWaitOnIntake());
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.setGoToIntake(false);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
    }
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
