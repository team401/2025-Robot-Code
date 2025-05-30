package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoScore extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  @AutoLogOutput(key = "AutoScore/location")
  private DesiredLocation currentScoringLocation;

  @AutoLogOutput(key = "AutoScore/target")
  private FieldTarget currentFieldTarget;

  private GamePiece gamePiece;

  public AutoScore(
      Drive drive,
      ScoringSubsystem scoring,
      GamePiece piece,
      DesiredLocation scoringLocation,
      FieldTarget target) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.currentScoringLocation = scoringLocation;
    this.currentFieldTarget = target;
    this.gamePiece = piece;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    if (drive != null) {
      drive.setDriveLinedUp(false);
      drive.setGoToIntake(false);
      drive.setDesiredLocation(currentScoringLocation);
      drive.fireTrigger(DriveTrigger.BeginLinear);
    }

    if (scoringSubsystem != null) {
      scoringSubsystem.setGamePiece(gamePiece);
      scoringSubsystem.setTarget(currentFieldTarget);
    }
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);
      System.out.println("AutoScore canceled lineup!");
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (drive != null) {
      Logger.recordOutput("AutoScore/driveFinished", drive.isDriveAlignmentFinished());
    }
    if (scoringSubsystem != null) {
      Logger.recordOutput("AutoScore/scoringFinished", !scoringSubsystem.shouldWaitOnScore());
    }

    return (drive == null || drive.isDriveAlignmentFinished())
        && (scoringSubsystem == null || !scoringSubsystem.shouldWaitOnScore());
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
