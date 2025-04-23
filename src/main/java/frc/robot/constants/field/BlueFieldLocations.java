package frc.robot.constants.field;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.subsystems.drive.Drive.DesiredLocation;

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

  public Translation2d blueAutoLineTranslation = new Translation2d(7.5, 4);
  public Rotation2d blueAutoLineRotation = new Rotation2d(Math.toRadians(-180));

  public Translation2d blueNetScoreTranslation = new Translation2d(7.253, 5.021);
  public Rotation2d blueNetScoreRotation = new Rotation2d(Math.toRadians(-180));

  public Translation2d blueReefOTF0Translation = new Translation2d(6.56, 4);
  public Rotation2d blueReefOTF0Rotation = new Rotation2d(Math.toRadians(-180));

  public Translation2d blueAlgaeOTF0Translation = new Translation2d(5.86, 4.018);

  public Translation2d blueReefOTF1Translation = new Translation2d(6.56, 4);
  public Rotation2d blueReefOTF1Rotation = new Rotation2d(Math.toRadians(-180));

  public Translation2d blueReefOTF2Translation = new Translation2d(5.5, 2.3);
  public Rotation2d blueReefOTF2Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d blueReefOTF3Translation = new Translation2d(5.5, 2.3);
  public Rotation2d blueReefOTF3Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d blueReefOTF4Translation = new Translation2d(3.5, 2.5);
  public Rotation2d blueReefOTF4Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d blueReefOTF5Translation = new Translation2d(3.5, 2.5);
  public Rotation2d blueReefOTF5Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d blueReefOTF6Translation = new Translation2d(2.64, 4);
  public Rotation2d blueReefOTF6Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d blueReefOTF7Translation = new Translation2d(2.64, 4);
  public Rotation2d blueReefOTF7Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d blueReefOTF8Translation = new Translation2d(3.5, 5.6);
  public Rotation2d blueReefOTF8Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d blueReefOTF9Translation = new Translation2d(3.5, 5.6);
  public Rotation2d blueReefOTF9Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d blueReefOTF10Translation = new Translation2d(5.5, 5.6);
  public Rotation2d blueReefOTF10Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d blueReefOTF11Translation = new Translation2d(5.5, 5.6);
  public Rotation2d blueReefOTF11Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d blueReef0Translation = new Translation2d(5.285, 4.186);
  public Translation2d blueReef1Translation = new Translation2d(5.285, 3.861);

  public Translation2d blueAlgae0Translation = new Translation2d(5.167, 4.018);

  public Translation2d blueReef2Translation = new Translation2d(5.024, 3.408);
  public Translation2d blueReef3Translation = new Translation2d(4.742, 3.241);

  public Translation2d blueAlgae1Translation = new Translation2d(4.825, 3.442);

  public Translation2d blueReef4Translation = new Translation2d(4.252, 3.264);
  public Translation2d blueReef5Translation = new Translation2d(3.952, 3.389);

  public Translation2d blueAlgae2Translation = new Translation2d(4.15, 3.434);

  public Translation2d blueReef6Translation = new Translation2d(3.637, 3.859);
  public Translation2d blueReef7Translation = new Translation2d(3.637, 4.186);

  public Translation2d blueAlgae3Translation = new Translation2d(3.82, 4.023);

  public Translation2d blueReef8Translation = new Translation2d(3.936, 4.635);
  public Translation2d blueReef9Translation = new Translation2d(4.245, 4.813);

  public Translation2d blueAlgae4Translation = new Translation2d(4.124, 4.613);

  public Translation2d blueReef10Translation = new Translation2d(4.745, 4.791);
  public Translation2d blueReef11Translation = new Translation2d(5.022, 4.647);

  public Translation2d blueAlgae5Translation = new Translation2d(4.29, 4.609);

  public Translation2d blueCoralStationRightTranslation = new Translation2d(1.3, 1);
  public Rotation2d blueCoralStationRightRotation = new Rotation2d(Units.degreesToRadians(60));

  public Translation2d blueCoralStationLeftTranslation = new Translation2d(1.2, 7);
  public Rotation2d blueCoralStationLeftRotation = new Rotation2d(Units.degreesToRadians(-60));

  public Translation2d blueNetTranslation = new Translation2d(7.459, 6.284);
  public Rotation2d blueNetRotation = new Rotation2d(0);

  // left stack center (1.2, 5.85)
  public Translation2d coralAlgaeStackLeftTopCorner = new Translation2d(1.45, 6.1);
  public Translation2d coralAlgaeStackLeftBottomCorner = new Translation2d(0.95, 5.6);
  // middle stack center (1.2, 4.05)
  public Translation2d coralAlgaeStackMiddleTopCorner = new Translation2d(1.45, 4.3);
  public Translation2d coralAlgaeStackMiddleBottomCorner = new Translation2d(0.95, 3.8);
  // right stack center (1.2, 2.2)
  public Translation2d coralAlgaeStackRightTopCorner = new Translation2d(1.45, 2.45);
  public Translation2d coralAlgaeStackRightBottomCorner = new Translation2d(0.95, 1.95);

  public Translation2d findPoleTranslationFromReefLocation(DesiredLocation location) {
    switch (location) {
      case Reef0:
        return blueReef0Translation;
      case Reef1:
        return blueReef1Translation;
      case Reef2:
        return blueReef2Translation;
      case Reef3:
        return blueReef3Translation;
      case Reef4:
        return blueReef4Translation;
      case Reef5:
        return blueReef5Translation;
      case Reef6:
        return blueReef6Translation;
      case Reef7:
        return blueReef7Translation;
      case Reef8:
        return blueReef8Translation;
      case Reef9:
        return blueReef9Translation;
      case Reef10:
        return blueReef10Translation;
      case Reef11:
        return blueReef11Translation;
      default:
        System.out.println(
            "ERROR: Tried to find reef pole translation for non-reef-pole DesiredLocation "
                + location.name());
        return Translation2d.kZero;
    }
  }

  public Translation2d findAlgaeTranslationFromReefLocation(DesiredLocation location) {
    switch (location) {
      case Algae0:
        return blueAlgae0Translation;
      case Algae1:
        return blueAlgae1Translation;
      case Algae2:
        return blueAlgae2Translation;
      case Algae3:
        return blueAlgae3Translation;
      case Algae4:
        return blueAlgae4Translation;
      case Algae5:
        return blueAlgae5Translation;
      default:
        System.out.println(
            "ERROR: Tried to find reef algae translation for non-reef-algae DesiredLocation "
                + location.name());
        return Translation2d.kZero;
    }
  }

  public DesiredLocation getClosestCoralStation(Pose2d robotPose) {
    double leftDistance = robotPose.getTranslation().getDistance(blueCoralStationLeftTranslation);
    double rightDistance = robotPose.getTranslation().getDistance(blueCoralStationRightTranslation);

    if (leftDistance < rightDistance) {
      return DesiredLocation.CoralStationLeft;
    } else {
      return DesiredLocation.CoralStationRight;
    }
  }
}
