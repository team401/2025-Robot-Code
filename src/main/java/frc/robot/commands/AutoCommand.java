package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class AutoCommand extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  private DesiredLocation currentScoringLocation;
  private int scoringLocationIndex = 0;
  private boolean shouldGoToIntake = false;

  public AutoCommand(Drive drive, ScoringSubsystem scoring) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.currentScoringLocation = JsonConstants.autoPath.scoringLocations.get(scoringLocationIndex);

    addRequirements(drive, scoringSubsystem);
  }

  public void initialize() {
    drive.setDesiredLocation(currentScoringLocation);
    drive.setDesiredIntakeLocation(JsonConstants.autoPath.intakeLocation);
    drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextPath() {
    return drive.isDriveAlignmentFinished()
        && (shouldGoToIntake
            ? scoringSubsystem.shouldWaitOnIntake()
            : scoringSubsystem.shouldWaitOnIntake());
  }

  public void prepareDrive() {
    drive.setGoToIntake(shouldGoToIntake);

    // if intake is false, go to next scoring location
    if (!shouldGoToIntake) {
      scoringLocationIndex++;
      currentScoringLocation = JsonConstants.autoPath.scoringLocations.get(scoringLocationIndex);
      drive.setDesiredLocation(currentScoringLocation);
    }

    drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
  }

  public void execute() {
    if (isReadyForNextPath()) { // if alignment finished, we move on to next location
      // toggle intake
      shouldGoToIntake = !shouldGoToIntake;
      prepareDrive();
    }
  }
}
