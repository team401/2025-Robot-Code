package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import org.littletonrobotics.junction.AutoLogOutput;

public class AutoScore extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  @AutoLogOutput(key = "AutoScore/location")
  private DesiredLocation currentScoringLocation;

  public AutoScore(Drive drive, ScoringSubsystem scoring, DesiredLocation scoringLocation) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.currentScoringLocation = scoringLocation;

    if (drive != null && scoringSubsystem != null) {
      // addRequirements(drive, scoringSubsystem);
    } else if (drive != null) {
      // addRequirements(drive);
    } else if (scoringSubsystem != null) {
      // addRequirements(scoringSubsystem);
    }
  }

  public void initialize() {
    if (drive != null) {
      drive.setGoToIntake(false);
      drive.setDesiredLocation(currentScoringLocation);
      drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
      drive.alignToFieldElement();
    }
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    return false;
    // return (drive == null || drive.isDriveAlignmentFinished())
    //     && (scoringSubsystem == null || !scoringSubsystem.shouldWaitOnScore());
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
