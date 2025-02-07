package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.strategies.AutoIntake;
import frc.robot.commands.strategies.AutoScore;
import frc.robot.constants.AutoPath;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem;

public class AutoCommand extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  private DesiredLocation currentScoringLocation;
  private int scoringLocationIndex = 0;
  private boolean shouldGoToIntake = false;
  private AutoPath path = null;
  private AutoIntake intakeCommand = null;
  private AutoScore scoreCommand = null;

  public AutoCommand(Drive drive, ScoringSubsystem scoring, AutoPath path) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.path = path;
    this.currentScoringLocation = path.scoringLocations.get(scoringLocationIndex);

    addRequirements(drive, scoringSubsystem);
  }

  public void initialize() {
    intakeCommand = new AutoIntake(drive, scoringSubsystem, path.intakeLocation);
    scoreCommand = new AutoScore(drive, scoringSubsystem, currentScoringLocation);
  }

  /**
   * checks if we have another path
   *
   * @return true if scoringLocationIndex + 1 is less than the size of locations
   */
  public boolean hasNextPath() {
    return path.scoringLocations.size() > (scoringLocationIndex);
  }

  public void execute() {
    if (shouldGoToIntake) {
      if (intakeCommand.isFinished()) { // if we have finished intake, time to score
        shouldGoToIntake = !shouldGoToIntake;
        if (!hasNextPath()) { // if there is no scoring location left, we cancel
          this.cancel();
        }
      } else if (!intakeCommand
          .isScheduled()) { // if we are wanting to intake and not scheduled yet, schedule
        intakeCommand.schedule();
      }
    } else { // wanting to score
      if (scoreCommand.isFinished()) { // if scoring is finished, intake again
        shouldGoToIntake = !shouldGoToIntake;
        scoringLocationIndex++;
        if (hasNextPath()) { // if there is a path, go ahead and get new command for next time
          // around
          currentScoringLocation = path.scoringLocations.get(scoringLocationIndex);
          scoreCommand = new AutoScore(drive, scoringSubsystem, currentScoringLocation);
        }
      } else if (!scoreCommand.isScheduled()
          && hasNextPath()) { // if its not scheduled and we have a path, run score
        scoreCommand.schedule();
      } else { // we have no score left, time to intake one last time
        shouldGoToIntake = !shouldGoToIntake;
      }
    }
  }
}
