package frc.robot.constants;

public class JsonConstants {

  public static void loadConstants() {

    DrivetrainConstants.synced.saveData();
    DrivetrainConstants.synced.loadData();
    ElevatorConstants.synced.loadData();
    ElevatorConstants.Sim.synced.loadData();

    drivetrainConstants = DrivetrainConstants.synced.getObject();
    elevatorConstants = ElevatorConstants.synced.getObject();
    elevatorConstantsSim = ElevatorConstants.Sim.synced.getObject();
  }

  public static DrivetrainConstants drivetrainConstants;
  public static ElevatorConstants elevatorConstants;
  public static ElevatorConstants.Sim elevatorConstantsSim;
}
