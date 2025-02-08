package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;

public class RampConstants {
    public static final JSONSync<RampConstants> synced =
      new JSONSync<RampConstants>(
          new RampConstants(),
          "RampConstants.json",
            EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());
    
    public final Double intakePosition = 1.0;
    public final Double climbPosition = 2.5;

    public final Integer motorId = 1;

}
