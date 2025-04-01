package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
  public static final JSONSync<VisionConstants> synced =
      new JSONSync<VisionConstants>(
          new VisionConstants(),
          "VisionConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Integer FrontLeftCameraIndex = 0;
  public final Integer FrontRightCameraIndex = 0;
  public final Transform3d FrontLeftTransform = new Transform3d();
  public final Transform3d FrontRightTransform =
      new Transform3d(
          Units.inchesToMeters(7.0),
          Units.inchesToMeters(-5.5),
          Units.inchesToMeters(12.0),
          new Rotation3d(0, 0, 0));

  public final Integer safetyThreshold = 10;
}
