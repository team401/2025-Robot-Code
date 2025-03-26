package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

public class DriveConstants {

  public static final JSONSync<DriveConstants> synced =
      new JSONSync<DriveConstants>(
          new DriveConstants(),
          "DriveConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Double kDriveToPointTranslationP = 10.0;
  public final Double kDriveToPointTranslationI = 0.0;
  public final Double kDriveToPointTranslationD = 0.0;
  public final Double kDriveTranslationMaxVelocity = 5.0;
  public final Double kDriveTranslationMaxAcceleration = 5.0;
  public final Double kPositionTolerance = 0.005;
  public final Double kVelocityTolerance = 0.005;

  public final Double kDriveToPointHeadingP = 10.0;
  public final Double kDriveToPointHeadingI = 0.0;
  public final Double kDriveToPointHeadingD = 0.0;
  public final Double kDriveHeadingMaxVelocity = 5.0;
  public final Double kDriveHeadingMaxAcceleration = 5.0;
  public final Double kAngleTolerance = 0.005;
  public final Double kAngularVelocityTolerance = 0.005;

  public final Double lineupErrorMargin = 0.05;
}
