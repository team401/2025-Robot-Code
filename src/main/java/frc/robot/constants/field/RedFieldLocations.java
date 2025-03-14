package frc.robot.constants.field;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

  public Translation2d redReefCenterTranslation = new Translation2d(12.5, 4.0259);
  public Rotation2d redReefCenterRotation = new Rotation2d();

  public Translation2d redReefOTF0Translation = new Translation2d(11.5, 4.1);
  public Rotation2d redReefOTF0Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d redReefOTF1Translation = new Translation2d(11.5, 4.1);
  public Rotation2d redReefOTF1Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d redReefOTF2Translation = new Translation2d(12.2, 5.3);
  public Rotation2d redReefOTF2Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d redReefOTF3Translation = new Translation2d(12.2, 5.3);
  public Rotation2d redReefOTF3Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d redReefOTF4Translation = new Translation2d(14, 5.5);
  public Rotation2d redReefOTF4Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d redReefOTF5Translation = new Translation2d(14, 5.5);
  public Rotation2d redReefOTF5Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d redReefOTF6Translation = new Translation2d(14.7, 4);
  public Rotation2d redReefOTF6Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d redReefOTF7Translation = new Translation2d(14.7, 4);
  public Rotation2d redReefOTF7Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d redReefOTF8Translation = new Translation2d(14, 2.5);
  public Rotation2d redReefOTF8Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d redReefOTF9Translation = new Translation2d(14, 2.5);
  public Rotation2d redReefOTF9Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d redReefOTF10Translation = new Translation2d(12.2, 2.6);
  public Rotation2d redReefOTF10Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d redReefOTF11Translation = new Translation2d(12.2, 2.6);
  public Rotation2d redReefOTF11Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d redCoralStationRightTranslation = new Translation2d(16.3, 7);
  public Rotation2d redCoralStationRightRotation = new Rotation2d(Units.degreesToRadians(-120));

  public Translation2d redCoralStationLeftTranslation = new Translation2d(16.3, 1);
  public Rotation2d redCoralStationLeftRotation = new Rotation2d(Units.degreesToRadians(120));

  public Translation2d redNetTranslation = new Translation2d(10.319, 2.768);
  public Rotation2d redNetRotation = new Rotation2d(Math.PI / 2);

  // left stacck center (16.2, 2.2)
  public Translation2d coralAlgaeStackLeftTopCorner = new Translation2d(15.95, 1.95);
  public Translation2d coralAlgaeStackLeftBottomCorner = new Translation2d(16.45, 2.45);
  // middle stack center (16.2, 4.05)
  public Translation2d coralAlgaeStackMiddleTopCorner = new Translation2d(15.95, 3.8);
  public Translation2d coralAlgaeStackMiddleBottomCorner = new Translation2d(16.45, 4.3);
  // right stackc center (16.2, 5.85)
  public Translation2d coralAlgaeStackRightTopCorner = new Translation2d(15.95, 5.6);
  public Translation2d coralAlgaeStackRightBottomCorner = new Translation2d(16.45, 6.1);
}
