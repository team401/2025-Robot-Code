package frc.robot.commands.drive;

import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drive.DesiredLocation;

public class DesiredLocationSelector {
  private static int locationIndex;

  public static void setLocationFromIndex(Drive drive) {
    switch (locationIndex) {
      case 0:
        drive.setDesiredLocation(DesiredLocation.Reef0);
        return;
      case 1:
        drive.setDesiredLocation(DesiredLocation.Reef1);
        return;
      case 2:
        drive.setDesiredLocation(DesiredLocation.Reef2);
        return;
      case 3:
        drive.setDesiredLocation(DesiredLocation.Reef3);
        return;
      case 4:
        drive.setDesiredLocation(DesiredLocation.Reef4);
        return;
      case 5:
        drive.setDesiredLocation(DesiredLocation.Reef5);
        return;
      case 6:
        drive.setDesiredLocation(DesiredLocation.Reef6);
        return;
      case 7:
        drive.setDesiredLocation(DesiredLocation.Reef7);
        return;
      case 8:
        drive.setDesiredLocation(DesiredLocation.Reef8);
        return;
      case 9:
        drive.setDesiredLocation(DesiredLocation.Reef9);
        return;
      case 10:
        drive.setDesiredLocation(DesiredLocation.Reef10);
        return;
      case 11:
        drive.setDesiredLocation(DesiredLocation.Reef11);
        return;
      case 12:
        drive.setDesiredLocation(DesiredLocation.ProcessorLeft);
        return;
      case 13:
        drive.setDesiredLocation(DesiredLocation.ProcessorRight);
        return;
      default:
        drive.setDesiredLocation(DesiredLocation.Reef0);
    }
  }

  public static void incrementIndex() {
    if (locationIndex > 12) {
      locationIndex = 0;
    } else {
      locationIndex++;
    }
  }

  public static void decrementIndex() {
    if (locationIndex < 1) {
      locationIndex = 13;
    } else {
      locationIndex++;
    }
  }
}
