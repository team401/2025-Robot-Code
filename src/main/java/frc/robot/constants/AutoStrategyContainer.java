package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;
import java.util.LinkedList;
import java.util.List;

public class AutoStrategyContainer {
  public List<AutoStrategy> strategies = new LinkedList<>();

  public AutoStrategyContainer(String[] strategyNames) {
    for (int i = 0; i < strategyNames.length; i++) {
      JSONSync<AutoStrategy> synced =
          new JSONSync<AutoStrategy>(
              new AutoStrategy(),
              Filesystem.getDeployDirectory()
                  .toPath()
                  .resolve("constants/autoStrategies/" + strategyNames[i] + ".json")
                  .toString(),
              new JSONSyncConfigBuilder().setPrettyPrinting(true).build());
      strategies.add(synced.getObject());
    }
  }
}
