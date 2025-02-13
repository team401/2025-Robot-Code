package frc.robot.constants;

import static edu.wpi.first.units.Units.Degrees;

import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;

public class ClimbConstants {

  public static final JSONSync<ClimbConstants> synced =
      new JSONSync<ClimbConstants>(
          new ClimbConstants(),
          "ClimbConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Angle restingAngle = Degrees.of(20);
  public final Angle searchingAngle = Degrees.of(60);
  public final Angle finalHangingAngle = Degrees.of(0);

  public final Integer leadClimbMotorId = 0;
  public final Integer followerClimbMotorId = 1;

  public final Double climbkS = 160.0;
  public final Double climbkV = 1.0;
  public final Double climbkA = 1.0;
  public final Double climbkG = 1.0;
  public final Double climbP = 50.0;
  public final Double climbI = 0.0;
  public final Double climbD = 0.0;

  public final Double climbCurrentLimit = 60.0;

  public final Boolean invertFollowerClimbMotor = false;

  public static final class Sim {
    public static final JSONSync<ClimbConstants.Sim> synced =
        new JSONSync<ClimbConstants.Sim>(
            new ClimbConstants.Sim(),
            "ClimbConstants.Sim.json",
            EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
            new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

    public final Double climbArmLengthMeters = 0.5;
    public final Double climbArmMassKg = 0.5;
    public final Double climbGearing = 75.0;
    public final Double climbStdDevs = 0.005;

    public final Double minAngleRads = 0.0;
    public final Double maxAngleRads = 2 * Math.PI;
    public final Double startAngleRads = 0.0;

    public final Boolean simGravity = false;
  }
}
