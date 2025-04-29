package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.drive.Drive.DesiredLocation;

public class DesiredLocationUtil {
  public static Pose2d findGoalPoseFromDesiredLocation(
      DesiredLocation desiredLocation, boolean isAllianceRed) {
    switch (desiredLocation) {
        // NOTE: pairs of reef sides (ie 0 and 1) will have the same otf pose (approximately 0.5-1
        // meter away from center of tag)
      case Reef0:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF0Translation,
                JsonConstants.redFieldLocations.redReefOTF0Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF0Translation,
                JsonConstants.blueFieldLocations.blueReefOTF0Rotation);
      case Reef1:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF1Translation,
                JsonConstants.redFieldLocations.redReefOTF1Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF1Translation,
                JsonConstants.blueFieldLocations.blueReefOTF1Rotation);
      case Algae0:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redAlgaeOTF0Translation,
                JsonConstants.redFieldLocations.redReefOTF1Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueAlgaeOTF0Translation,
                JsonConstants.blueFieldLocations.blueReefOTF1Rotation);
      case Reef2:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF2Translation,
                JsonConstants.redFieldLocations.redReefOTF2Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF2Translation,
                JsonConstants.blueFieldLocations.blueReefOTF2Rotation);
      case Reef3:
      case Algae1:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF3Translation,
                JsonConstants.redFieldLocations.redReefOTF3Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF3Translation,
                JsonConstants.blueFieldLocations.blueReefOTF3Rotation);
      case Reef4:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF4Translation,
                JsonConstants.redFieldLocations.redReefOTF4Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF4Translation,
                JsonConstants.blueFieldLocations.blueReefOTF4Rotation);
      case Reef5:
      case Algae2:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF5Translation,
                JsonConstants.redFieldLocations.redReefOTF5Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF5Translation,
                JsonConstants.blueFieldLocations.blueReefOTF5Rotation);
      case Reef6:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF6Translation,
                JsonConstants.redFieldLocations.redReefOTF6Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF6Translation,
                JsonConstants.blueFieldLocations.blueReefOTF6Rotation);
      case Reef7:
      case Algae3:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF7Translation,
                JsonConstants.redFieldLocations.redReefOTF7Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF7Translation,
                JsonConstants.blueFieldLocations.blueReefOTF7Rotation);
      case Reef8:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF8Translation,
                JsonConstants.redFieldLocations.redReefOTF8Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF8Translation,
                JsonConstants.blueFieldLocations.blueReefOTF8Rotation);
      case Reef9:
      case Algae4:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF9Translation,
                JsonConstants.redFieldLocations.redReefOTF9Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF9Translation,
                JsonConstants.blueFieldLocations.blueReefOTF9Rotation);
      case Reef10:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redReefOTF10Translation,
                JsonConstants.redFieldLocations.redReefOTF10Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF10Translation,
                JsonConstants.blueFieldLocations.blueReefOTF10Rotation);
      case Reef11:
      case Algae5:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redAlgae5OTFTranslation,
                JsonConstants.redFieldLocations.redReefOTF11Rotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueReefOTF11Translation,
                JsonConstants.blueFieldLocations.blueReefOTF11Rotation);
      case CoralStationRight:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redCoralStationRightTranslation,
                JsonConstants.redFieldLocations.redCoralStationRightRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueCoralStationRightTranslation,
                JsonConstants.blueFieldLocations.blueCoralStationRightRotation);
      case CoralStationLeft:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redCoralStationLeftTranslation,
                JsonConstants.redFieldLocations.redCoralStationLeftRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueCoralStationLeftTranslation,
                JsonConstants.blueFieldLocations.blueCoralStationLeftRotation);
      case AutoLine:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redAutoLineTranslation,
                JsonConstants.redFieldLocations.redAutoLineRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueAutoLineTranslation,
                JsonConstants.blueFieldLocations.blueAutoLineRotation);
      case NetScore:
        return isAllianceRed
            ? new Pose2d(
                JsonConstants.redFieldLocations.redNetScoreTranslation,
                JsonConstants.redFieldLocations.redNetScoreRotation)
            : new Pose2d(
                JsonConstants.blueFieldLocations.blueNetScoreTranslation,
                JsonConstants.blueFieldLocations.blueNetScoreRotation);
      default:
        System.out.println(
            "WARNING: Unknown field location "
                + desiredLocation
                + " in DesiredLocationUtil.findGoalPoseFromDesiredLocation");
        return null;
    }
  }

  public static Rotation2d getRotationForReefSide(Drive drive) {
    switch (drive.getDesiredLocation()) {
      case Reef0:
      case Reef1:
      case Algae0:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF0Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF0Rotation;
      case Reef2:
      case Reef3:
      case Algae1:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF2Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF2Rotation;
      case Reef4:
      case Reef5:
      case Algae2:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF4Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF4Rotation;
      case Reef6:
      case Reef7:
      case Algae3:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF6Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF6Rotation;
      case Reef8:
      case Reef9:
      case Algae4:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF8Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF8Rotation;
      case Reef10:
      case Reef11:
      case Algae5:
        return drive.isAllianceRed()
            ? JsonConstants.redFieldLocations.redReefOTF10Rotation
            : JsonConstants.blueFieldLocations.blueReefOTF10Rotation;
      default:
        return new Rotation2d();
    }
  }
}
