package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;

public class AutoIntake extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  private DesiredLocation intakeLocation;

  private FieldTarget intakeFieldTarget;

  private GamePiece gamePiece;

  public AutoIntake(
      Drive drive,
      ScoringSubsystem scoring,
      GamePiece piece,
      DesiredLocation intakeLocation,
      FieldTarget target) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.intakeLocation = intakeLocation;
    this.intakeFieldTarget = target;
    this.gamePiece = piece;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    if (drive != null) {
      if (gamePiece == GamePiece.Coral) {
        drive.setDesiredIntakeLocation(intakeLocation);
        drive.setGoToIntake(true);
        drive.fireTrigger(DriveTrigger.BeginOTF);
      } else {
        drive.setDesiredLocation(intakeLocation);
        drive.setGoToIntake(false);
        drive.fireTrigger(DriveTrigger.BeginLinear);
      }
    }

    if (scoringSubsystem != null) {
      scoringSubsystem.setGamePiece(gamePiece);
      scoringSubsystem.setTarget(intakeFieldTarget);
      scoringSubsystem.fireTrigger(ScoringTrigger.BeginIntake);
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (scoringSubsystem == null && drive != null) {
      return drive.isDriveAlignmentFinished();
    }

    return (scoringSubsystem == null || !scoringSubsystem.shouldWaitOnIntake());
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.setGoToIntake(false);
      drive.fireTrigger(DriveTrigger.CancelAutoAlignment);

      System.out.println("AutoIntake canceled lineup!");
    }
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
