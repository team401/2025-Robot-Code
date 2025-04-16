package frc.robot.constants;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

public final class OperatorConstants {
  @JSONExclude
  public static final JSONSync<OperatorConstants> synced =
      new JSONSync<OperatorConstants>(
          new OperatorConstants(),
          "OperatorConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public String mappingFile = "controller.json";
}
