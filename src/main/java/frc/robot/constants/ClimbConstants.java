package frc.robot.constants;

import static edu.wpi.first.units.Units.Radians;

import coppercore.parameter_tools.json.JSONExclude;
import coppercore.parameter_tools.json.JSONSync;
import coppercore.parameter_tools.json.JSONSyncConfigBuilder;
import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Filesystem;

public class ClimbConstants {

  public static final JSONSync<ClimbConstants> synced =
      new JSONSync<ClimbConstants>(
          new ClimbConstants(),
          "ClimbConstants.json",
          EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
          new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

  public final Angle restingAngle = Radians.of(0);
  public final Angle searchingAngle = Radians.of(60);
  public final Angle finalHangingAngle = Radians.of(0);

  public final Integer leadClimbMotorId = 16;
  public final Integer followerClimbMotorId = 17;

  public final Double climbkS = 0.0;
  public final Double climbkV = 0.0;
  public final Double climbkA = 0.0;
  public final Double climbkG = 0.0;
  public final Double climbkP = 5.0;
  public final Double climbkI = 0.0;
  public final Double climbkD = 0.0;

  public final Double climbCurrentLimit = 60.0;

  public final Boolean invertFollowerClimbMotor = true;

  public static final class Sim {
    @JSONExclude
    public static final JSONSync<ClimbConstants.Sim> synced =
        new JSONSync<ClimbConstants.Sim>(
            new ClimbConstants.Sim(),
            Filesystem.getDeployDirectory()
                .toPath()
                .resolve("constants/ClimbConstants.Sim.json")
                .toString(),
            new JSONSyncConfigBuilder().build());

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
