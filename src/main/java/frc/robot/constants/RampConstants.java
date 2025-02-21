package frc.robot.constants;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;

import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Time;

public class RampConstants {
  public static final JSONSync<RampConstants> synced =
      new JSONSync<RampConstants>(
          new RampConstants(),
          "RampConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  

  public final Integer motorId = 1;
  public final Double positionRange = 0.02;

  public final Double PID_TalonFX_P = 0.0;
  public final Double PID_TalonFX_I = 0.0;
  public final Double PID_TalonFX_D = 0.0;

  public final Double PID_SIM_P = 7.5;
  public final Double PID_SIM_I = 0.001;
  public final Double PID_SIM_D = 0.0;


  //TODO: Tune Values

  //Positions
  public final Double idlePosition = 1.0;
  public final Double intakePosition = 1.0;
  public final Double climbPosition = 2.5;

  //Voltages
  public final Double intakeVoltage = 0.4; 
  public final Double homingVoltage = 0.4;

  //Homing Settings
  public final Time homingMaxTime = Seconds.of(5);
  public final Time homingMaxUnmovingTime = Seconds.of(5);
  public final AngularVelocity homingVelocityThresholdMetersPerSecond = RadiansPerSecond.of(0.01);
  public final Integer homingVelocityFilterWindowSize = 50;
}
