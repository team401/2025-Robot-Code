package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.wpilibj.Filesystem;

public class BlueFieldLocations {
  public static final JSONSync<BlueFieldLocations> synced =
      new JSONSync<BlueFieldLocations>(
          new BlueFieldLocations(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/BlueFieldLocations.json")
              .toString(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());
}
