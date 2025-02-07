package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.constants.field.BlueFieldLocations;
import frc.robot.constants.field.RedFieldLocations;
import frc.robot.constants.subsystems.DrivetrainConstants;
import frc.robot.constants.subsystems.ElevatorConstants;

public class JsonConstants {

  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    DrivetrainConstants.synced.loadData();
    ElevatorConstants.synced.loadData();
    ElevatorConstants.Sim.synced.loadData();
    ClawConstants.synced.loadData();
    ClawConstants.Sim.synced.loadData();
    ScoringFeatureFlags.synced.loadData();
    ScoringSetpoints.synced.loadData();
    RedFieldLocations.synced.loadData();
    BlueFieldLocations.synced.loadData();
    DrivetrainConstants.synced.loadData();
    VisionConstants.synced.loadData();

    elevatorConstants = ElevatorConstants.synced.getObject();
    elevatorConstantsSim = ElevatorConstants.Sim.synced.getObject();
    clawConstants = ClawConstants.synced.getObject();
    clawConstantsSim = ClawConstants.Sim.synced.getObject();
    scoringFeatureFlags = ScoringFeatureFlags.synced.getObject();
    scoringSetpoints = ScoringSetpoints.synced.getObject();
    redFieldLocations = RedFieldLocations.synced.getObject();
    blueFieldLocations = BlueFieldLocations.synced.getObject();
    drivetrainConstants = DrivetrainConstants.synced.getObject();
    visionConstants = VisionConstants.synced.getObject();
  }

  public static ElevatorConstants elevatorConstants;
  public static ElevatorConstants.Sim elevatorConstantsSim;
  public static ClawConstants clawConstants;
  public static ClawConstants.Sim clawConstantsSim;
  public static ScoringFeatureFlags scoringFeatureFlags;
  public static ScoringSetpoints scoringSetpoints;
  public static RedFieldLocations redFieldLocations;
  public static BlueFieldLocations blueFieldLocations;
  public static DrivetrainConstants drivetrainConstants;
  public static VisionConstants visionConstants;
}
