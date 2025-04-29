package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.Logger;

/** Automatically intake the reef0 algae (barge side) from the reef */
public class AutoIntakeBargeAlgae extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;

  private DesiredLocation driveTarget;
  private FieldTarget scoringTarget;

  public AutoIntakeBargeAlgae(
      Drive drive, ScoringSubsystem scoring, DesiredLocation driveLocation) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
    this.driveTarget = driveLocation;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    if (drive != null) {
      drive.setDriveLinedUp(false);
      drive.setGoToIntake(true);
      drive.setShouldLinearDriveSlowly(true);
      drive.setDesiredLocation(driveTarget);
      drive.setDesiredIntakeLocation(driveTarget);
      drive.fireTrigger(DriveTrigger.BeginLinear);
    }

    if (scoringSubsystem != null) {
      scoringSubsystem.setGamePiece(GamePiece.Algae);
      scoringSubsystem.setAlgaeIntakeTarget(FieldTarget.L2);
      scoringSubsystem.setTarget(FieldTarget.L2);
      scoringSubsystem.fireTrigger(ScoringTrigger.CancelWarmup);
      scoringSubsystem.fireTrigger(ScoringTrigger.BeginIntake);
    }
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.fireTrigger(DriveTrigger.CancelLinear);
      drive.setShouldLinearDriveSlowly(false);
      System.out.println("AutoIntakeBargeAlgae canceled lineup!");
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (drive != null) {
      Logger.recordOutput(
          "AutoIntakeBargeAlgae/driveFinished", drive.isDriveCloseToFinalLineupPose());
    }

    if (scoringSubsystem == null) {
      return (drive == null || drive.isDriveCloseToFinalLineupPose());
    } else {
      Logger.recordOutput("AutoIntakeBargeAlgae/hasAlgae", scoringSubsystem.isAlgaeDetected());
    }

    return scoringSubsystem == null || scoringSubsystem.isAlgaeDetected();
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
