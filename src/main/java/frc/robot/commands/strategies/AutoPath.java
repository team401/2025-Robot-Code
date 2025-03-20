package frc.robot.commands.strategies;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.FileVersionException;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DriveTrigger;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import java.io.IOException;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;

public class AutoPath extends Command {
  private Drive drive;
  private ScoringSubsystem scoringSubsystem;
  private Command autoRun = null;

  public AutoPath(Drive drive, ScoringSubsystem scoring, String path) {
    this.drive = drive;
    this.scoringSubsystem = scoring;
  }

  public void initialize() {
    if (drive != null) {
      drive.setGoToIntake(false);
      drive.fireTrigger(DriveTrigger.BeginPathFollow);
    }

    Logger.recordOutput("pathName", Drive.getPathToRun());

    PathPlannerPath path = null;
    try {
      PathPlannerPath.fromPathFile(Drive.getPathToRun());
      autoRun = AutoBuilder.followPath(path);
      autoRun.schedule();
    } catch (FileVersionException | IOException | ParseException e) {
      e.printStackTrace();
    }
  }

  /**
   * determines when everything has finished (scoring / intake and drive at location)
   *
   * @return true if we are ready for next path
   */
  public boolean isReadyForNextAction() {
    return autoRun == null || autoRun.isFinished();
  }

  public void end(boolean interrupted) {
    if (drive != null) {
      drive.setGoToIntake(false);
      drive.setGoalSpeeds(new ChassisSpeeds(), true);
      drive.fireTrigger(DriveTrigger.CancelPathFollow);

      System.out.println("AutoPath ended!");
    }
  }

  public boolean isFinished() {
    return isReadyForNextAction();
  }
}
