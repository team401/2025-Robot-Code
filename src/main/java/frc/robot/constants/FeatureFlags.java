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

  public final Boolean runElevator = false;
  public final Boolean runClimb = true;
  public final Boolean runDrive = true;
  public Boolean runVision = true;
  public final Boolean runScoring = true;
  // public final Boolean runElevator = true; // TODO: Figure out if we need Mechanism-level feature
  // flags
  // public final Boolean runClaw = true;
}
