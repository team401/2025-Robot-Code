package frc.robot.constants;

import frc.robot.constants.subsystems.DrivetrainConstants;
import frc.robot.constants.subsystems.ElevatorConstants;

public class JsonConstants {

  public static void loadConstants() {

    DrivetrainConstants.synced.loadData();
    ElevatorConstants.synced.loadData();
    ElevatorConstants.Sim.synced.loadData();
    FieldConstants.synced.loadData();

    drivetrainConstants = DrivetrainConstants.synced.getObject();
    elevatorConstants = ElevatorConstants.synced.getObject();
    elevatorConstantsSim = ElevatorConstants.Sim.synced.getObject();
    fieldConstants = FieldConstants.synced.getObject();
  }

  public static DrivetrainConstants drivetrainConstants;
  public static ElevatorConstants elevatorConstants;
  public static ElevatorConstants.Sim elevatorConstantsSim;
  public static FieldConstants fieldConstants;
}
