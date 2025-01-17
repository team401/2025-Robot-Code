package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
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
          new JSONSync.JSONSyncConfigBuilder().build());

  public Boolean runElevator = true;
  public Boolean runDrive = true;
}
