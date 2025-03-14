package frc.robot.constants;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class RampConstants {
  public static final JSONSync<RampConstants> synced =
      new JSONSync<RampConstants>(
          new RampConstants(),
          "RampConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  // PID
  public final Double PID_TalonFX_P = 5.0;
  public final Double PID_TalonFX_I = 0.0;
  public final Double PID_TalonFX_D = 0.0;

  public final Double PID_SIM_P = 7.5;
  public final Double PID_SIM_I = 0.001;
  public final Double PID_SIM_D = 0.0;

  // TODO: Tune Values

  // Positions
  public final Double idlePosition = 1.0;
  public final Double intakePosition = 1.0;
  public final Double climbPosition = 2.5;
  public final Double maxElevatorSafePosition = 3.0; // TODO: real value
  public final Double extendPosition = 1.0;

  // Auto homing
  public final Double autoHomePositionHigh = 2.0;
  public final Double autoHomePositionLow = -0.1;

  // Voltages
  public final Double intakeVoltage = -3.0;
  public final Double intakeHoldVoltage = -1.0;
  public final Double climbVoltage = 0.0;
  public final Double climbHoldVoltage = 0.5;
  public final Double homingVoltage = -2.0;
  public final Double extendingVoltage = 1.0;

  // Homing Settings
  public final Time homingMaxTime = Seconds.of(2);
  public final Time homingMaxUnmovingTime = Seconds.of(0.2);
  public final AngularVelocity homingVelocityThresholdMetersPerSecond = RadiansPerSecond.of(0.1);
  public final Integer homingVelocityFilterWindowSize = 5;

  // Other
  public final Boolean inverted = false;
  public final Integer motorId = 1;
  public final Double positionRange = 0.05;
}
