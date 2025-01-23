package frc.robot.commands.drive;

import frc.robot.subsystems.drive.Drive;

public class DesiredLocationSelector {
  private static int locationIndex;

  public static void setLocationFromIndex(Drive drive) {
    drive.setDesiredLocation(locationIndex);
  }

  public static void incrementIndex() {
    if (locationIndex > 13) {
      locationIndex = 0;
    } else {
      locationIndex++;
    }
  }

  public static void decrementIndex() {
    if (locationIndex < 1) {
      locationIndex = 14;
    } else {
      locationIndex++;
    }
  }
}
