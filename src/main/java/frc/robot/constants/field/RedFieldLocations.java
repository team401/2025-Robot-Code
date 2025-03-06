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

  public Translation2d redReefCenterTranslation = new Translation2d(12.5, 4.0259);
  public Rotation2d redReefCenterRotation = new Rotation2d();

  public Translation2d redReef01Translation = new Translation2d(11.5, 4.1);
  public Rotation2d redReef01Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d redReef23Translation = new Translation2d(12.2, 5.3);
  public Rotation2d redReef23Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d redReef45Translation = new Translation2d(14, 5.5);
  public Rotation2d redReef45Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d redReef67Translation = new Translation2d(14.7, 4);
  public Rotation2d redReef67Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d redReef89Translation = new Translation2d(14, 2.5);
  public Rotation2d redReef89Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d redReef1011Translation = new Translation2d(12.2, 2.6);
  public Rotation2d redReef1011Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d redCoralStationRightTranslation = new Translation2d(15.82, 7.32);
  public Rotation2d redCoralStationRightRotation = new Rotation2d(-2.2689);

  public Translation2d redCoralStationLeftTranslation = new Translation2d(15.82, 0.6048);
  public Rotation2d redCoralStationLeftRotation = new Rotation2d(2.2689);

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
