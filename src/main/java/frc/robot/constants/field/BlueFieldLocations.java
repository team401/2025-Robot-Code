package frc.robot.constants.field;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public static Translation2d blueReefCenterTranslation = new Translation2d(4.489323, 4.0259);
  public static Rotation2d blueReefCenterRotation = new Rotation2d();

  public static Translation2d blueReefAprilTag1Translation = new Translation2d(193.10, 130.17);
  public static Rotation2d blueReefAprilTag1Rotation = new Rotation2d(Math.toRadians(300));

  public static Translation2d blueReefAprilTag2Translation = new Translation2d(209.49, 158.5);
  public static Rotation2d blueReefAprilTag2Rotation = new Rotation2d(Math.toRadians(0));

  public static Translation2d blueReefAprilTag3Translation = new Translation2d(193.10, 186.83);
  public static Rotation2d blueReefAprilTag3Rotation = new Rotation2d(Math.toRadians(60));

  public static Translation2d blueReefAprilTag4Translation = new Translation2d(160.39, 186.83);
  public static Rotation2d blueReefAprilTag4Rotation = new Rotation2d(Math.toRadians(120));

  public static Translation2d blueReefAprilTag5Translation = new Translation2d(144.00, 158.5);
  public static Rotation2d blueReefAprilTag5Rotation = new Rotation2d(Math.toRadians(180));

  public static Translation2d blueReefAprilTag6Translation = new Translation2d(160.39, 130.17);
  public static Rotation2d blueReefAprilTag6Rotation = new Rotation2d(Math.toRadians(240));
}
