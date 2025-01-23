package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import coppercore.parameter_tools.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;

public final class Platform {
  @JSONExclude
  public static final JSONSync<Platform> synced =
      new JSONSync<Platform>(
          new Platform(),
          Filesystem.getDeployDirectory().toPath().resolve("constants/Platform.json").toString(),
          new JSONSyncConfigBuilder().build());

  public final String platform = "testrig";
}
