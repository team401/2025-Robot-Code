package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;

public class AutoCommand extends Command {
  private Drive drive;

  private DesiredLocation currentScoringLocation;
  private int scoringLocationIndex = 0;
  private boolean shouldGoToIntake = false;

  public AutoCommand(Drive drive) {
    this.drive = drive;
    this.currentScoringLocation = JsonConstants.autoPath.scoringLocations.get(scoringLocationIndex);

    addRequirements(drive);
  }

  public void initialize() {
    drive.setDesiredLocation(currentScoringLocation);
    drive.setDesiredIntakeLocation(JsonConstants.autoPath.intakeLocation);
    drive.fireTrigger(DriveTrigger.BeginAutoAlignment);
  }

  public boolean isReadyForNextPath() {
    return drive.isDriveAlignmentFinished();
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
