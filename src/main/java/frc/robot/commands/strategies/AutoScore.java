package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class AutoScore extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  private DesiredLocation currentScoringLocation;

  public AutoScore(Drive drive, ScoringSubsystem scoring, DesiredLocation scoringLocation) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.currentScoringLocation = scoringLocation;

    addRequirements(drive, scoringSubsystem);
  }

  public void initialize() {
    if (drive != null) {
      drive.setDesiredLocation(currentScoringLocation);
      drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    return (drive != null && drive.isDriveAlignmentFinished())
        && (scoringSubsystem != null && !scoringSubsystem.shouldWaitOnScore());
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
