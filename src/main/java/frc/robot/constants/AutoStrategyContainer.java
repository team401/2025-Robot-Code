package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.Drive.DesiredLocation;
import frc.robot.subsystems.scoring.ScoringSubsystem.FieldTarget;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;
import java.io.File;
import java.util.LinkedList;
import java.util.List;

public class AutoStrategyContainer {
  public List<AutoStrategy> strategies = new LinkedList<>();

  public enum ActionType {
    Intake,
    Score
  };

  public static record Action(
      ActionType type, GamePiece piece, DesiredLocation location, FieldTarget scoringTarget) {}

  public AutoStrategyContainer(File[] strategyNames) {
    String[] fileNames = new String[strategyNames.length];
    for (int i = 0; i < strategyNames.length; i++) {
      fileNames[i] = strategyNames[i].getName();
      JSONSync<AutoStrategy> synced =
          new JSONSync<AutoStrategy>(
              new AutoStrategy(),
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("auto/" + strategyNames[i].getName())
                  .toString(),
              new JSONSyncConfigBuilder().setPrettyPrinting(true).build());
      AutoStrategy strat = synced.getObject();
      strat.autoStrategyName = strategyNames[i].getName().replace(".json", "");
      strategies.add(strat);
    }
  }

  public List<AutoStrategy> getStrategies() {
    return strategies;
  }
}
