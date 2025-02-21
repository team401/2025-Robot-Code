package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

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
  public final Double positionRange = 0.02;

  public final Double PID_TalonFX_P = 0.0;
  public final Double PID_TalonFX_I = 0.0;
  public final Double PID_TalonFX_D = 0.0;

  public final Double PID_SIM_P = 7.5;
  public final Double PID_SIM_I = 0.001;
  public final Double PID_SIM_D = 0.0;

  public final double idlePosition = 1.0;

  // TODO: tune this value
  public final double intakeVoltage = 0.4; 
}
