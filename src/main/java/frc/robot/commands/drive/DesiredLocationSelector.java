package frc.robot.commands.drive;

import frc.robot.subsystems.drive.Drive;

public class DesiredLocationSelector {
  private static int locationIndex;

  public static void setLocationFromIndex(Drive drive) {
    drive.setDesiredLocation(locationIndex);
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
