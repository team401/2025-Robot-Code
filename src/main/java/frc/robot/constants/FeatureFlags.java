package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import coppercore.parameter_tools.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;

public final class FeatureFlags {
  @JSONExclude
  public static final JSONSync<FeatureFlags> synced =
      new JSONSync<FeatureFlags>(
          new FeatureFlags(),
          "FeatureFlags.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public Boolean runElevator = false;
  public Boolean runDrive = true;
}
