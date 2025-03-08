package frc.robot.subsystems.drive;

import frc.robot.constants.JsonConstants;
import frc.robot.subsystems.scoring.ScoringSubsystem;
import frc.robot.subsystems.scoring.ScoringSubsystem.GamePiece;

/** Provides utilities for finding the reef tag ID and camera ID to use for lineup */
public class ReefLineupUtil {
  /**
   * gets tag to use for final alignment with vision
   *
   * @return int representing tag id to use
   */
  public static int getTagIdForReef(Drive drive) {
    boolean allianceRed = drive.isAllianceRed();
    switch (drive.getDesiredLocation()) {
      case Reef0:
      case Reef1:
      case Algae0:
        return allianceRed ? 10 : 21;
      case Reef2:
      case Reef3:
      case Algae1:
        return allianceRed ? 9 : 22;
      case Reef4:
      case Reef5:
      case Algae2:
        return allianceRed ? 8 : 17;
      case Reef6:
      case Reef7:
      case Algae3:
        return allianceRed ? 7 : 18;
      case Reef8:
      case Reef9:
      case Algae4:
        return allianceRed ? 6 : 19;
      case Reef10:
      case Reef11:
      case Algae5:
        return allianceRed ? 11 : 20;
      default:
        return -1;
    }
  }

  /**
   * gets camera index for vision single tag lineup
   *
   * @return 0 for Front Left camera; 1 for Front Right camera
   */
  public static int getCameraIndexForLineup(Drive drive) {
    switch (drive.getDesiredLocation()) {
        // Right Side of reef side (align to left camera)
      case Reef0:
      case Reef2:
      case Reef4:
      case Reef6:
      case Reef8:
      case Reef10:
      case Algae0:
      case Algae1:
      case Algae2:
      case Algae3:
      case Algae4:
      case Algae5:
        return JsonConstants.visionConstants.FrontLeftCameraIndex;
        // Left side of reef side (align to right camera)
      case Reef1:
      case Reef3:
      case Reef5:
      case Reef7:
      case Reef9:
      case Reef11:
        return JsonConstants.visionConstants.FrontRightCameraIndex;
      default:
        return -1;
    }
  }

  /**
   * gets cross track offset for lineup
   *
   * @param cameraIndex camera to check offset
   * @return offset for camera
   */
  public static Double getCrossTrackOffset(int cameraIndex) {
    if (cameraIndex == JsonConstants.visionConstants.FrontRightCameraIndex) {
      if (ScoringSubsystem.getInstance() != null
          && ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral) {
        return JsonConstants.drivetrainConstants.driveCrossTrackFrontRightOffset; // coral
      }
      return JsonConstants.drivetrainConstants.driveCrossTrackFrontRightAlgaeOffset; // algae
    } else {
      if (ScoringSubsystem.getInstance() != null
          && ScoringSubsystem.getInstance().getGamePiece() == GamePiece.Coral) {
        return JsonConstants.drivetrainConstants.driveCrossTrackFrontLeftOffset; // coral
      }
      return JsonConstants.drivetrainConstants.driveCrossTrackFrontLeftAlgaeOffset; // algae
    }
  }
}
