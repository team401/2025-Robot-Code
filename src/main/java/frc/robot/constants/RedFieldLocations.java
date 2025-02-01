package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

public class RedFieldLocations {

  public static final JSONSync<RedFieldLocations> synced =
      new JSONSync<RedFieldLocations>(
          new RedFieldLocations(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/RedFieldLocations.json")
              .toString(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Pose2d reef0 =
      new Pose2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17), new Rotation2d());
}
