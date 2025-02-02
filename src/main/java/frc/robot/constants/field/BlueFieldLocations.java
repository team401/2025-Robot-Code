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

  public Translation2d blueReefCenterTranslation =
      new Translation2d(Units.inchesToMeters(176.75), Units.inchesToMeters(158.5));
  public Rotation2d blueReefCenterRotation = new Rotation2d();

  public Translation2d blueReefAprilTag1Translation =
      new Translation2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17));
  public Rotation2d blueReefAprilTag1Rotation = new Rotation2d(Math.toRadians(300));

  public Translation2d blueReefAprilTag2Translation =
      new Translation2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.5));
  public Rotation2d blueReefAprilTag2Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d blueReefAprilTag3Translation =
      new Translation2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83));
  public Rotation2d blueReefAprilTag3Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d blueReefAprilTag4Translation =
      new Translation2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83));
  public Rotation2d blueReefAprilTag4Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d blueReefAprilTag5Translation =
      new Translation2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.5));
  public Rotation2d blueReefAprilTag5Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d blueReefAprilTag6Translation =
      new Translation2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17));
  public Rotation2d blueReefAprilTag6Rotation = new Rotation2d(Math.toRadians(240));

  public Translation2d blueCoralStationRightTranslation = new Translation2d();
  public Rotation2d blueCoralStationRightRotation = new Rotation2d();

  public Translation2d blueCoralStationLeftTranslation = new Translation2d();
  public Rotation2d blueCoralStationLeftRotation = new Rotation2d();
}
