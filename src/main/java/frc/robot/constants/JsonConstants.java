package frc.robot.constants;

import coppercore.parameter_tools.path_provider.EnvironmentHandler;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.InitBindings;
import frc.robot.constants.field.BlueFieldLocations;
import frc.robot.constants.field.RedFieldLocations;
import frc.robot.constants.subsystems.DrivetrainConstants;
import frc.robot.constants.subsystems.ElevatorConstants;

public class JsonConstants {

  public static void loadConstants() {
    EnvironmentHandler.getEnvironmentHandler(
        Filesystem.getDeployDirectory().toPath().resolve("constants/config.json").toString());
    DrivetrainConstants.synced.saveData();
    InitBindings.synced.saveData();
    DrivetrainConstants.synced.loadData();
    ElevatorConstants.synced.loadData();
    ElevatorConstants.Sim.synced.loadData();
    RedFieldLocations.synced.loadData();
    BlueFieldLocations.synced.loadData();
    DrivetrainConstants.synced.loadData();
    InitBindings.synced.loadData();

    elevatorConstants = ElevatorConstants.synced.getObject();
    elevatorConstantsSim = ElevatorConstants.Sim.synced.getObject();
    redFieldLocations = RedFieldLocations.synced.getObject();
    blueFieldLocations = BlueFieldLocations.synced.getObject();
    drivetrainConstants = DrivetrainConstants.synced.getObject();
    initBindings = InitBindings.synced.getObject();
  }

  public static ElevatorConstants elevatorConstants;
  public static ElevatorConstants.Sim elevatorConstantsSim;
  public static RedFieldLocations redFieldLocations;
  public static BlueFieldLocations blueFieldLocations;
  public static DrivetrainConstants drivetrainConstants;
  public static InitBindings initBindings;
}
