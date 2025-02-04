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
      new Translation2d(176.75, (158.5));
  public Rotation2d blueReefCenterRotation = new Rotation2d();

  
  public Translation2d blueReef01Translation =
      new Translation2d(11.5, 4.1);
  public Rotation2d blueReef01Rotation = new Rotation2d(Math.toRadians(0));

  public Translation2d blueReef23Translation =
      new Translation2d(209.49, 158.5);
  public Rotation2d blueReef23Rotation = new Rotation2d(Math.toRadians(-60));

  public Translation2d blueReef45Translation =
      new Translation2d(193.10, 186.83);
  public Rotation2d blueReef45Rotation = new Rotation2d(Math.toRadians(120));

  public Translation2d blueReef67Translation =
      new Translation2d(160.39, 186.83);
  public Rotation2d blueReef67Rotation = new Rotation2d(Math.toRadians(180));

  public Translation2d blueReef89Translation =
      new Translation2d(144.00, 158.5);
  public Rotation2d blueReef89Rotation = new Rotation2d(Math.toRadians(-120));

  public Translation2d blueReef1011Translation =
      new Translation2d(160.39, 130.17);
  public Rotation2d blueReef1011Rotation = new Rotation2d(Math.toRadians(60));

  public Translation2d blueCoralStationRightTranslation = new Translation2d();
  public Rotation2d blueCoralStationRightRotation = new Rotation2d();

  public Translation2d blueCoralStationLeftTranslation = new Translation2d();
  public Rotation2d blueCoralStationLeftRotation = new Rotation2d();
}