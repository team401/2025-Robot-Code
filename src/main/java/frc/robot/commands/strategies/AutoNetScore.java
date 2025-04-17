package frc.robot.commands.strategies;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import frc.robot.subsystems.scoring.ScoringSubsystem.ScoringTrigger;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class AutoNetScore extends Command {
  private ScoringSubsystem scoringSubsystem;

  @AutoLogOutput(key = "AutoScore/location")
  private DesiredLocation currentScoringLocation;

  @AutoLogOutput(key = "AutoScore/target")
  private FieldTarget currentFieldTarget;

  public AutoNetScore(ScoringSubsystem scoring) {
    this.scoringSubsystem = scoring;
    // we dont want to require subsystems (it prevents drive otf from running)
  }

  public void initialize() {
    if (scoringSubsystem != null) {
      scoringSubsystem.setGamePiece(GamePiece.Algae);
      scoringSubsystem.setTarget(FieldTarget.Net);
      scoringSubsystem.fireTrigger(ScoringTrigger.StartWarmup);
    }
  }

  public void end(boolean interrupted) {
    scoringSubsystem.fireTrigger(ScoringTrigger.CancelWarmup);
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    if (scoringSubsystem != null) {
      Logger.recordOutput("AutoNetScore/scoringFinished", !scoringSubsystem.shouldWaitOnScore());
    }

    return scoringSubsystem == null || !scoringSubsystem.shouldWaitOnScore();
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
