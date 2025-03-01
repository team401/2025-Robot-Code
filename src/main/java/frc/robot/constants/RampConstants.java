package frc.robot.constants;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Time;

public class RampConstants {
  public static final JSONSync<RampConstants> synced =
      new JSONSync<RampConstants>(
          new RampConstants(),
          "RampConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Integer motorId = 18;
  public final Double positionRange = 0.2;

  public final Double PID_TalonFX_P = 0.2;
  public final Double PID_TalonFX_I = 0.0;
  public final Double PID_TalonFX_D = 0.0;

  public final Double PID_SIM_P = 7.5;
  public final Double PID_SIM_I = 0.001;
  public final Double PID_SIM_D = 0.0;

  // TODO: Tune Values

  // Positions
  public final Double idlePosition = 0.0;
  public final Double intakePosition = 0.0;
  public final Double climbPosition = 1.0;

  // Voltages
  public final Double intakeVoltage = -1.0;
  public final Double homingVoltage = -2.0;
  public final Boolean inverted = false;

  // Homing Settings
  public final Time homingMaxTime = Seconds.of(3);
  public final Time homingMaxUnmovingTime = Seconds.of(3);
  public final Angle homingVelocityThresholdRadiansPerSecond = Radians.of(0.1);
  public final Integer homingVelocityFilterWindowSize = 50;
}
