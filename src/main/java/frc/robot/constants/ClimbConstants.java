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

  public final int leadClimbMotorId = 0;
  public final int followerClimbMotorId = 1;

  public final double climbP = 5;
  public final double climbI = 0;
  public final double climbD = 0;

  public final double climbCurrentLimit = 60;

  public final boolean invertFollowerClimbMotor = false;

  public static final class Sim {
    public static final JSONSync<ClimbConstants.Sim> synced =
        new JSONSync<ClimbConstants.Sim>(
            new ClimbConstants.Sim(),
            "ClimbConstants.Sim.json",
            EnvironmentHandler.getEnvironmentHandler().getEnvironmentPathProvider(),
            new JSONSyncConfigBuilder().setPrettyPrinting(true).build());

    public final double climbArmLengthMeters = 0.5;
    public final double climbArmMassKg = 0.5;
    public final double climbGearing = 75;
    public final double climbStdDevs = 0.005;
  }
}
