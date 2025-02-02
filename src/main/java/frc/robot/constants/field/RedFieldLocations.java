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

  public Translation2d redReefCenterTranslation =
      new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
  public Rotation2d redReefCenterRotation = new Rotation2d();

  public Translation2d redReefAprilTag1Translation =
      new Translation2d(Units.inchesToMeters(530.49), Units.inchesToMeters(130.17));
  public Rotation2d redReefAprilTag1Rotation = new Rotation2d(Math.toRadians(300));

  public Translation2d redReefAprilTag2Translation =
      new Translation2d(Units.inchesToMeters(546.87), Units.inchesToMeters(158.5));
  public Rotation2d redReefAprilTag2Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d redReefAprilTag3Translation =
      new Translation2d(Units.inchesToMeters(530.49), Units.inchesToMeters(186.83));
  public Rotation2d redReefAprilTag3Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d redReefAprilTag4Translation =
      new Translation2d(Units.inchesToMeters(497.77), Units.inchesToMeters(186.83));
  public Rotation2d redReefAprilTag4Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d redReefAprilTag5Translation =
      new Translation2d(Units.inchesToMeters(481.39), Units.inchesToMeters(158.5));
  public Rotation2d redReefAprilTag5Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d redReefAprilTag6Translation =
      new Translation2d(Units.inchesToMeters(497.77), Units.inchesToMeters(130.17));
  public Rotation2d redReefAprilTag6Rotation = new Rotation2d(Math.toRadians(240));

  public Translation2d redCoralStationRightTranslation = new Translation2d();
  public Rotation2d redCoralStationRightRotation = new Rotation2d();

  public Translation2d redCoralStationLeftTranslation = new Translation2d();
  public Rotation2d redCoralStationLeftRotation = new Rotation2d();
}
