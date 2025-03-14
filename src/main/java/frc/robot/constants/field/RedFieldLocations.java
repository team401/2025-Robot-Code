package frc.robot.constants.field;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.Drive.DesiredLocation;

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

  public Translation2d redReef0Translation = new Translation2d(12.273, 3.867);
  public Translation2d redReef1Translation = new Translation2d(12.273, 4.187);

  public Translation2d redReef23Translation = new Translation2d(12.2, 5.3);
  public Rotation2d redReef23Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d redReef2Translation = new Translation2d(12.519, 4.621);
  public Translation2d redReef3Translation = new Translation2d(12.811, 4.817);

  public Translation2d redReef45Translation = new Translation2d(14, 5.5);
  public Rotation2d redReef45Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d redReef4Translation = new Translation2d(13.305, 4.794);
  public Translation2d redReef5Translation = new Translation2d(13.605, 4.632);

  public Translation2d redReef67Translation = new Translation2d(14.7, 4);
  public Rotation2d redReef67Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d redReef6Translation = new Translation2d(13.864, 4.202);
  public Translation2d redReef7Translation = new Translation2d(13.864, 3.867);

  public Translation2d redReef89Translation = new Translation2d(14, 2.5);
  public Rotation2d redReef89Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d redReef8Translation = new Translation2d(13.6, 3.43);
  public Translation2d redReef9Translation = new Translation2d(13.317, 3.222);

  public Translation2d redReef1011Translation = new Translation2d(12.2, 2.6);
  public Rotation2d redReef1011Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d redReef10Translation = new Translation2d(12.837, 3.256);
  public Translation2d redReef11Translation = new Translation2d(12.546, 3.428);

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

  public Translation2d findPoleTranslationFromReefLocation(DesiredLocation location) {
    switch (location) {
      case Reef0:
        return redReef0Translation;
      case Reef1:
        return redReef1Translation;
      case Reef2:
        return redReef2Translation;
      case Reef3:
        return redReef3Translation;
      case Reef4:
        return redReef4Translation;
      case Reef5:
        return redReef5Translation;
      case Reef6:
        return redReef6Translation;
      case Reef7:
        return redReef7Translation;
      case Reef8:
        return redReef8Translation;
      case Reef9:
        return redReef9Translation;
      case Reef10:
        return redReef10Translation;
      case Reef11:
        return redReef11Translation;
      default:
        System.out.println(
            "ERROR: Tried to find reef pole translation for non-reef-pole DesiredLocation "
                + location.name());
        return Translation2d.kZero;
    }
  }
}
