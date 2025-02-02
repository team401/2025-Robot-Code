package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

public class ScoringFeatureFlags {
  @JSONExclude
  public static final JSONSync<ScoringFeatureFlags> synced =
      new JSONSync<ScoringFeatureFlags>(
          new ScoringFeatureFlags(),
          "ScoringFeatureFlags.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Boolean runElevator = true;
  public final Boolean runClaw = true;
  public final Boolean runWrist = true;
}
