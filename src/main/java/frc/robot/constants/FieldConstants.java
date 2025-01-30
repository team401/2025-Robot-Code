package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Filesystem;

/** List of all POI positions on the field */
public final class FieldConstants {
  @JSONExclude
  public static final JSONSync<FieldConstants> synced =
      new JSONSync<FieldConstants>(
          new FieldConstants(),
          Filesystem.getDeployDirectory()
              .toPath()
              .resolve("constants/FieldConstants.json")
              .toString(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  // RED - Pose2d
  public final Pose2d redReefCenter =
      new Pose2d(514.13, 158.5, new Rotation2d()); // center of red apriltags
  public static final Pose2d redReefAprilTag1 =
      new Pose2d(530.49, 130.17, new Rotation2d(300)); // real april tag 6
  public final Pose2d redReefAprilTag2 =
      new Pose2d(546.87, 158.5, new Rotation2d(0)); // real april tag 7
  public final Pose2d redReefAprilTag3 =
      new Pose2d(530.49, 186.83, new Rotation2d(60)); // real april tag 8
  public final Pose2d redReefAprilTag4 =
      new Pose2d(497.77, 186.83, new Rotation2d(120)); // real april tag 9
  public final Pose2d redReefAprilTag5 =
      new Pose2d(481.39, 158.5, new Rotation2d(180)); // real april tag 10
  public final Pose2d redReefAprilTag6 =
      new Pose2d(497.77, 130.17, new Rotation2d(240)); // real april tag 11

  // BLUE - Pose2d
  public final Pose2d blueReefCenter =
      new Pose2d(176.745, 158.5, new Rotation2d()); // center of blue apriltags
  public final Pose2d blueReefAprilTag1 =
      new Pose2d(193.10, 130.17, new Rotation2d(300)); // real april tag 22
  public final Pose2d blueReefAprilTag2 =
      new Pose2d(209.49, 158.5, new Rotation2d(0)); // real april tag 21
  public final Pose2d blueReefAprilTag3 =
      new Pose2d(193.10, 186.83, new Rotation2d(60)); // real april tag 20
  public final Pose2d blueReefAprilTag4 =
      new Pose2d(160.39, 186.83, new Rotation2d(120)); // real april tag 19
  public final Pose2d blueReefAprilTag5 =
      new Pose2d(144.00, 158.5, new Rotation2d(180)); // real april tag 18
  public final Pose2d blueReefAprilTag6 =
      new Pose2d(160.39, 130.17, new Rotation2d(240)); // real april tag 17
}
