package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;

public final class FeatureFlags {
  @JSONExclude
  public static final JSONSync<FeatureFlags> synced =
      new JSONSync<FeatureFlags>(
          new FeatureFlags(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/FeatureFlags.json")
              .toString(),
          new JSONSyncConfigBuilder().build());

  public Boolean runElevator = true;
  public Boolean runDrive = true;
}
