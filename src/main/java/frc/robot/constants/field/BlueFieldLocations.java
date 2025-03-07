package frc.robot.constants.field;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
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

  public Translation2d blueReefCenterTranslation = new Translation2d(4.5, 4);
  public Rotation2d blueReefCenterRotation = new Rotation2d();

  public Translation2d blueReef01Translation = new Translation2d(6.56, 4);
  public Rotation2d blueReef01Rotation = new Rotation2d(Math.toRadians(-180));

  public Translation2d blueReef23Translation = new Translation2d(5.5, 2.3);
  public Rotation2d blueReef23Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d blueReef45Translation = new Translation2d(3.5, 2.5);
  public Rotation2d blueReef45Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d blueReef67Translation = new Translation2d(2.64, 4);
  public Rotation2d blueReef67Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d blueReef89Translation = new Translation2d(3.5, 5.6);
  public Rotation2d blueReef89Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d blueReef1011Translation = new Translation2d(5.5, 5.6);
  public Rotation2d blueReef1011Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d blueCoralStationRightTranslation = new Translation2d(1.3, 1);
  public Rotation2d blueCoralStationRightRotation = new Rotation2d(Units.degreesToRadians(60));

  public Translation2d blueCoralStationLeftTranslation = new Translation2d(1.2, 7);
  public Rotation2d blueCoralStationLeftRotation = new Rotation2d(Units.degreesToRadians(-60));

  // left stack center (1.2, 5.85)
  public Translation2d coralAlgaeStackLeftTopCorner = new Translation2d(1.45, 6.1);
  public Translation2d coralAlgaeStackLeftBottomCorner = new Translation2d(0.95, 5.6);
  // middle stack center (1.2, 4.05)
  public Translation2d coralAlgaeStackMiddleTopCorner = new Translation2d(1.45, 4.3);
  public Translation2d coralAlgaeStackMiddleBottomCorner = new Translation2d(0.95, 3.8);
  // right stack center (1.2, 2.2)
  public Translation2d coralAlgaeStackRightTopCorner = new Translation2d(1.45, 2.45);
  public Translation2d coralAlgaeStackRightBottomCorner = new Translation2d(0.95, 1.95);
}
