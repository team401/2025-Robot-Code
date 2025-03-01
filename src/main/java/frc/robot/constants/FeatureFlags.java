package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

public final class FeatureFlags {
  @JSONExclude
  public static final JSONSync<FeatureFlags> synced =
      new JSONSync<FeatureFlags>(
          new FeatureFlags(),
          "FeatureFlags.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public Boolean runRamp = true;
  public final Boolean runClimb = true;
  public final Boolean runDrive = true;
  public Boolean runVision = true;
  public final Boolean runScoring = true;
}
