package frc.robot.constants;

import coppercore.parameter_tools.JSONExclude;
import coppercore.parameter_tools.JSONSync;
import coppercore.parameter_tools.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

public final class OperatorConstants {
  @JSONExclude
  public static final JSONSync<OperatorConstants> synced =
      new JSONSync<OperatorConstants>(
          new OperatorConstants(),
          "OperatorConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public Integer kDriverControllerPort = 2;
  public Integer kLeftJoystickPort = 0;
  public Integer kRightJoystickPort = 1;
}
