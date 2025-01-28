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

// ﻿﻿﻿﻿﻿﻿ Error at java.base/java.io.FileOutputStream.open0(Native Method): Unhandled exception
// instantiating robot java.io.FileOutputStream java.io.FileNotFoundException:
// /home/lvuser/deploy/constants/DrivetrainConstants.json (Permission denied) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileOutputStream.open0(Native Method) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileOutputStream.open(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileOutputStream.<init>(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileOutputStream.<init>(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at java.base/java.io.FileWriter.<init>(Unknown Source) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at coppercore.parameter_tools.JSONSync.getFileWriter(JSONSync.java:126) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at coppercore.parameter_tools.JSONSync.saveData(JSONSync.java:50) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.constants.JsonConstants.loadConstants(JsonConstants.java:7) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.Robot.<init>(Robot.java:70) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.runRobot(RobotBase.java:370) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at edu.wpi.first.wpilibj.RobotBase.startRobot(RobotBase.java:510) ﻿
// ﻿﻿﻿﻿﻿﻿ 	at frc.robot.Main.main(Main.java:19) ﻿
// ﻿﻿﻿﻿﻿﻿  ﻿
