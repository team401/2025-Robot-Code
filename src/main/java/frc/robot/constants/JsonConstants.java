package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;

public class JsonConstants {

  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());

    DrivetrainConstants.synced.loadData();
    ElevatorConstants.synced.loadData();
    ElevatorConstants.Sim.synced.loadData();
    RedFieldLocations.synced.loadData();

    drivetrainConstants = DrivetrainConstants.synced.getObject();
    elevatorConstants = ElevatorConstants.synced.getObject();
    elevatorConstantsSim = ElevatorConstants.Sim.synced.getObject();
    redFieldLocations = RedFieldLocations.synced.getObject();
  }

  public static DrivetrainConstants drivetrainConstants;
  public static ElevatorConstants elevatorConstants;
  public static ElevatorConstants.Sim elevatorConstantsSim;
  public static RedFieldLocations redFieldLocations;
}
