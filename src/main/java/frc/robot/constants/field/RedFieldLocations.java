package frc.robot.constants.field;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public static Translation2d redReefCenterTranslation = new Translation2d(13.058902, 4.0259);
  public static Rotation2d redReefCenterRotation = new Rotation2d();

  public static Translation2d redReefAprilTag1Translation = new Translation2d(530.49, 130.17);
  public static Rotation2d redReefAprilTag1Rotation = new Rotation2d(Math.toRadians(300));

  public static Translation2d redReefAprilTag2Translation = new Translation2d(546.87, 158.5);
  public static Rotation2d redReefAprilTag2Rotation = new Rotation2d(Math.toRadians(0));

  public static Translation2d redReefAprilTag3Translation = new Translation2d(530.49, 186.83);
  public static Rotation2d redReefAprilTag3Rotation = new Rotation2d(Math.toRadians(60));

  public static Translation2d redReefAprilTag4Translation = new Translation2d(497.77, 186.83);
  public static Rotation2d redReefAprilTag4Rotation = new Rotation2d(Math.toRadians(120));

  public static Translation2d redReefAprilTag5Translation = new Translation2d(481.39, 158.5);
  public static Rotation2d redReefAprilTag5Rotation = new Rotation2d(Math.toRadians(180));

  public static Translation2d redReefAprilTag6Translation = new Translation2d(497.77, 130.17);
  public static Rotation2d redReefAprilTag6Rotation = new Rotation2d(Math.toRadians(240));
}
